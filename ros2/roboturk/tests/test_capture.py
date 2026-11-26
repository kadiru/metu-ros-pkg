from dataclasses import replace
import math
import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env
from roboturk.capture import CaptureEnv, CaptureTeacher
from roboturk.capture_contract import capture_action, capture_command, capture_observation
from roboturk.model import Ball, Command, Config, Robot, RobotKey, World


def test_capture_gym_contract_and_seed_replay():
    env = CaptureEnv()
    check_env(env, skip_render_check=True)
    first, _ = env.reset(seed=123)
    a = env.step(np.zeros(4, np.float32))
    second, _ = env.reset(seed=123)
    b = env.step(np.zeros(4, np.float32))
    np.testing.assert_array_equal(first, second)
    np.testing.assert_array_equal(a[0], b[0])
    assert a[1:] == b[1:]


def test_capture_requires_dribbler_and_sustained_control():
    env = CaptureEnv()
    env.reset(seed=0)
    key = env.key
    env.world = env.backend.reset(
        replace(env.world, robots=(Robot(key, 0, 0),), ball=Ball(0.115, 0))
    )
    for _ in range(25):
        _, _, done, _, _ = env.step(np.zeros(4))
        assert not done and env.dwell == 0
    action = np.array([0, 0, 0, 1], np.float32)
    for _ in range(19):
        _, _, done, _, _ = env.step(action)
        assert not done
    _, _, done, truncated, info = env.step(action)
    assert done and not truncated and info["is_success"] and info["hold_steps"] == 20


def test_capture_contract_rotates_with_the_world():
    cfg, key = Config(), RobotKey("blue", 0)
    original = World(0, (Robot(key, 1, 1, 0, 0.2, 0.1),), Ball(1.7, 1.3, 0.3, 0.1))
    rotated = World(0, (Robot(key, -1, 1, math.pi / 2, -0.1, 0.2),), Ball(-1.3, 1.7, -0.1, 0.3))
    np.testing.assert_allclose(
        capture_observation(original, key, cfg), capture_observation(rotated, key, cfg), atol=1e-6
    )


def test_capture_action_never_kicks_and_rejects_nonfinite():
    cfg = Config()
    assert capture_command([1, 1, 1, 1], cfg).kick == 0
    with pytest.raises(ValueError):
        capture_command([0, 0, float("nan"), 1], cfg)
    c = Command(0.3, -0.2, 0.5, 0.8, 4)
    restored = capture_command(capture_action(c, cfg), cfg)
    assert restored.vx == pytest.approx(c.vx) and restored.kick == 0


def test_teacher_faces_ball_behind_before_driving():
    env = CaptureEnv()
    env.reset(seed=0)
    env.world = replace(env.world, robots=(Robot(env.key, 0, 0),), ball=Ball(-1, 0.1))
    cmd = CaptureTeacher().command(env.world, env.key, env.intent, env.cfg)
    assert abs(cmd.vx) < 0.01 and abs(cmd.vy) < 0.01 and cmd.omega > 0
