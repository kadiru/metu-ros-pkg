"""Synthetic contract checks; native qualification is a separate recorded run."""

from dataclasses import replace
import gymnasium as gym
import numpy as np
import pytest
from roboturk.backend import ReferenceBackend
from roboturk.model import Ball, Config, Referee
from roboturk.decision_play import DecisionEnv, Decision, OPTIONS, encode, decode


class Fixed:
    def __init__(self, obs, actions, value=None):
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (obs,), dtype=np.float32)
        self.action_space = gym.spaces.Box(-1, 1, (actions,), dtype=np.float32)
        self.value = np.zeros(actions, np.float32) if value is None else np.array(value, np.float32)
        self.calls = 0

    def predict(self, obs, deterministic=True):
        assert obs.shape == self.observation_space.shape
        self.calls += 1
        return self.value.copy(), None


def env():
    cfg = Config()
    models = {
        "transfer": Fixed(70, 7, [0.1, 0, 0, -1, 0.1, 0, 0]),
        "finish": Fixed(36, 4, [0.2, 0, 0, 1]),
        "defender": Fixed(45, 3),
        "keeper": Fixed(45, 3),
    }
    task = DecisionEnv(ReferenceBackend(cfg), cfg, models)
    task.reset(seed=3200000)
    return task, models


def test_manager_contract_and_geometry_are_finite():
    e, _ = env()
    assert e.play.observation(e.world).shape == (75,)
    for i in range(3):
        d = Decision(i, 1, (0.5, -1.0), (3.0, 0.2), (e.world.attack_sign * 4.5, 0.4))
        a = encode(d, e.world, e.cfg)
        out = decode(a, e.world, e.cfg)
        assert out.option == i and out.owner == 1
        np.testing.assert_allclose(out.receive, d.receive, atol=1e-6)
        np.testing.assert_allclose(out.parking, d.parking, atol=1e-6)


@pytest.mark.parametrize("mode", range(3))
def test_no_scripted_target_or_phase_planner_during_inference(monkeypatch, mode):
    e, _ = env()

    def forbidden(*args, **kwargs):
        raise AssertionError("Scripted planner used")

    monkeypatch.setattr("roboturk.self_play.update_receiving_target", forbidden)
    monkeypatch.setattr("roboturk.self_play.MobileFinishPlan.update", forbidden)
    monkeypatch.setattr("roboturk.defended.receiving_point", forbidden)
    d = Decision(mode, 1, (0.5, 1), (3.1, 0), (e.world.attack_sign * 4.5, 0.3))
    e.play.commands(e.world, encode(d, e.world, e.cfg))
    assert e.play.decision.option == mode


def test_reception_does_not_force_handoff_and_model_can_switch_before_it():
    e, _ = env()
    for received in (False, True):
        e.play.states[0].received = received
        e.play.dwell = [0, 30] if received else [0, 0]
        for mode, owner in ((0, 0), (1, 0), (2, 1), (0, 1)):
            a = encode(
                Decision(mode, owner, (0, 2), (3, 0), (e.world.attack_sign * 4.5, 0.4)),
                e.world,
                e.cfg,
            )
            e.play.commands(e.world, a)
            assert (e.play.decision.option, e.play.decision.owner) == (mode, owner)


def test_dribble_mask_and_shoot_are_selected_by_manager():
    e, _ = env()
    for mode in (1, 2):
        a = encode(
            Decision(mode, 0, (0, 2), (3, 0), (e.world.attack_sign * 4.5, 0.4)), e.world, e.cfg
        )
        commands = e.play.commands(e.world, a)
        assert (commands[e.keys[0]].kick > 0) == (mode == 2)
        assert commands[e.keys[1]].vx == 0


def test_referee_halt_stops_without_manager_or_motion_calls():
    e, models = env()
    commands = e.play.commands(replace(e.world, referee=Referee.HALT))
    assert all(c.kick == c.vx == c.vy == 0 for c in commands.values())
    assert models["finish"].calls == models["transfer"].calls == 0


def test_nonfinite_decision_rejected_and_physics_rewind_rejected():
    e, _ = env()
    with pytest.raises(ValueError, match="finite"):
        decode(np.full(10, np.nan), e.world, e.cfg)
    with pytest.raises(ValueError, match="control period"):
        e.play.observe(e.world, [e.world], [])
    assert OPTIONS == ("pass", "dribble", "shoot")


@pytest.mark.parametrize("acquired", [False, True])
def test_receiving_touch_is_not_stable_acquisition(monkeypatch, acquired):
    e, _ = env()
    r = e.world.robot(e.keys[1])
    import math

    before = replace(
        e.world,
        ball=Ball(
            r.x + e.cfg.possession_x * math.cos(r.yaw), r.y + e.cfg.possession_x * math.sin(r.yaw)
        ),
    )
    after = replace(before, time=0.02, ball=Ball(r.x + 0.4, r.y + 0.4))
    e.world = before
    e.controlled = [False, acquired]
    e.play.dwell = [0, 19]
    monkeypatch.setattr(e.backend, "step", lambda commands: after)
    monkeypatch.setattr(e.backend, "last_substep_worlds", [after], raising=False)
    monkeypatch.setattr(e.backend, "last_kick_events", [], raising=False)
    a = encode(Decision(1, 1, (0, 2), (3, 0), (before.attack_sign * 4.5, 0.4)), before, e.cfg)
    _, _, done, _, info = e.step(a)
    assert (info["reason"] == "lost_possession") == acquired
    assert done == acquired
