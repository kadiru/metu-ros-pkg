from dataclasses import replace
import math
import numpy as np
from gymnasium.utils.env_checker import check_env
from roboturk.dribble import DribbleEnv
from roboturk.dribble_contract import dribble_observation
from roboturk.model import Ball, Config, Intent, Robot, RobotKey, Skill, World


def test_dribble_gym_seed_replay_and_attack_target():
    env = DribbleEnv()
    check_env(env, skip_render_check=True)
    first, _ = env.reset(seed=123)
    a = env.step(np.zeros(4, np.float32))
    second, _ = env.reset(seed=123)
    b = env.step(np.zeros(4, np.float32))
    np.testing.assert_array_equal(first, second)
    np.testing.assert_array_equal(a[0], b[0])
    assert a[1:] == b[1:]
    assert env.intent.target == (3.6 * env.world.attack_sign, 0)


def test_dribble_target_contract_is_rotation_equivariant():
    cfg, key = Config(), RobotKey("blue", 0)
    original = World(0, (Robot(key, 1, 1, 0, 0.2, 0.1),), Ball(1.7, 1.3, 0.3, 0.1))
    rotated = World(0, (Robot(key, -1, 1, math.pi / 2, -0.1, 0.2),), Ball(-1.3, 1.7, -0.1, 0.3))
    a = dribble_observation(original, key, Intent(Skill.DRIBBLE, (3.6, 0)), cfg)
    b = dribble_observation(rotated, key, Intent(Skill.DRIBBLE, (0, 3.6)), cfg)
    np.testing.assert_allclose(a, b, atol=1e-6)
    c = dribble_observation(original, key, Intent(Skill.DRIBBLE, (-3.6, 0)), cfg)
    assert not np.array_equal(a, c)


def test_dribble_fails_on_loss_between_control_frames(monkeypatch):
    env = DribbleEnv()
    env.reset(seed=0)
    held = replace(env.world, robots=(Robot(env.key, 0, 0),), ball=Ball(0.115, 0))
    lost = replace(held, ball=Ball(0.3, 0))
    env.world = held
    env.acquired, env.capture_step = True, 20
    env.backend.last_substep_worlds = [held, lost, held, held]
    monkeypatch.setattr(env.backend, "step", lambda _: held)
    _, _, done, truncated, info = env.step([0, 0, 0, 1])
    assert done and not truncated and not info["is_success"]
    assert info["reason"] == "lost_possession" and info["lost_possession_checks"] == 1


def test_dribble_requires_stationary_arrival_hold_and_active_dribbler():
    env = DribbleEnv()
    env.reset(seed=0)
    env.intent = Intent(Skill.DRIBBLE, (3.6, 0))
    held = replace(env.world, robots=(Robot(env.key, 3.485, 0),), ball=Ball(3.6, 0))
    env.world = env.backend.reset(held)
    env.acquired, env.capture_step, env.capture_distance = True, 20, 2.0
    for _ in range(49):
        _, _, done, _, info = env.step([0, 0, 0, 1])
        assert not done
    _, _, done, _, info = env.step([0, 0, 0, 1])
    assert done and info["is_success"] and info["arrival_hold_steps"] == 50
    env.reset(seed=0)
    env.world = env.backend.reset(held)
    env.acquired, env.capture_step = True, 20
    _, _, done, _, info = env.step([0, 0, 0, 0])
    assert done and info["reason"] == "lost_possession"
