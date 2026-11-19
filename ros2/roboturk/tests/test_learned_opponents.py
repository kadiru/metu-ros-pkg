from dataclasses import replace
from types import SimpleNamespace
import math
import numpy as np
import pytest
from roboturk.backend import ReferenceBackend
from roboturk.model import Config, Referee, World, Robot, Ball, world_vector
from roboturk.defended import DEFENDER, LaneDefender
from roboturk.keeper import KEEPER, ReactiveKeeper
from roboturk.learned_opponents import (
    OpponentController,
    LearnedOpponentPlayEnv,
    opponent_observation,
)


class ConstantModel:
    observation_space = SimpleNamespace(shape=(33,))
    action_space = SimpleNamespace(shape=(3,))

    def __init__(self, action):
        self.value = np.array(action, np.float32)
        self.calls = 0

    def predict(self, obs, deterministic=True):
        assert obs.shape == (33,)
        self.calls += 1
        return self.value, None


def test_every_opponent_command_comes_from_model(monkeypatch):
    def forbidden(*args):
        raise AssertionError("analytic opponent was called")

    monkeypatch.setattr(LaneDefender, "command", forbidden)
    monkeypatch.setattr(ReactiveKeeper, "command", forbidden)
    cfg = Config()
    d = OpponentController("defender", cfg, model=ConstantModel([0.2, -0.3, 0.4]))
    k = OpponentController("keeper", cfg, model=ConstantModel([-0.1, 0.3, -0.4]))
    env = LearnedOpponentPlayEnv(cfg, ReferenceBackend(cfg), d, k)
    env.reset(seed=1240042)
    for _ in range(10):
        env.backend.step({})
    assert d.model.calls == k.model.calls == 10
    evidence = env.backend.evidence()
    assert evidence["opponent"] == "learned_lane_v1"
    assert evidence["keeper"] == "learned_keeper_v1"
    assert evidence["defender_policy_calls"] == evidence["keeper_policy_calls"] == 10
    env.reset(seed=1240042)
    assert d.calls == k.calls == 0


def test_delay_acceleration_and_duplicate_frames():
    cfg = Config()
    c = OpponentController("keeper", cfg, model=ConstantModel([0, 1, 0.5]))
    c.delay, c.speed, c.acceleration = 2, 0.8, 2.0
    c.reset_control()
    w = World(0, (Robot(KEEPER, 4.32, 0, math.pi),), Ball(2, 0), Referee.RUNNING)
    commands = []
    for i in range(4):
        w = replace(w, time=i * cfg.dt)
        commands.append(c.command(w))
        assert c.command(w) == commands[-1]
    assert c.calls == 4
    assert commands[0].vy == commands[1].vy == 0
    vx, vy = world_vector(commands[2].vx, commands[2].vy, math.pi)
    assert vx == pytest.approx(0, abs=1e-8)
    assert vy == pytest.approx(cfg.dt * c.acceleration)
    assert commands[2].omega == pytest.approx(2)


def test_safety_gate_and_invalid_neural_action():
    c = OpponentController("defender", model=ConstantModel([np.nan, 0, 0]))
    w = World(0, (Robot(DEFENDER, 0, 0),), Ball(-1, 0), Referee.HALT)
    assert c.command(w).dribbler == 0
    assert c.model.calls == 0
    with pytest.raises(ValueError, match="Invalid learned"):
        c.command(replace(w, referee=Referee.RUNNING))


def test_geometry_features_rotate_with_field():
    c = OpponentController("defender", model=ConstantModel([0, 0, 0]))
    c.anchor_x, c.anchor_y, c.phase = 0.2, 0.1, 0.0
    w = World(
        0, (Robot(DEFENDER, 0.3, -0.2, 0.4, 0.1, -0.2),), Ball(-1, 0.2, 1.8, 0.1), Referee.RUNNING
    )
    a = opponent_observation(w, c)
    c.anchor_x, c.anchor_y = -c.anchor_x, -c.anchor_y
    r = w.robot(DEFENDER)
    rotated = replace(
        w,
        robots=(replace(r, x=-r.x, y=-r.y, yaw=r.yaw + math.pi, vx=-r.vx, vy=-r.vy),),
        ball=Ball(1, -0.2, -1.8, -0.1),
        attack_sign=-1,
    )
    b = opponent_observation(rotated, c)
    # Patrol phase is global context; rotate its phase too.
    c.phase = math.pi
    b = opponent_observation(rotated, c)
    assert np.allclose(a, b, atol=1e-6)
