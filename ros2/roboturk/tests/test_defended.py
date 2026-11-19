from dataclasses import replace
import math
import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env
from roboturk.model import Ball, Command
from roboturk.defended import DEFENDER, DefendedEnv, defended_observation
from roboturk.team import scoring_handoff


def test_defended_reset_replay_and_blocked_lane():
    env = DefendedEnv()
    check_env(env, skip_render_check=True)
    a, _ = env.reset(seed=10)
    first = env.step([0, 0, 0, -1, 0, 0, 0])
    b, _ = env.reset(seed=10)
    second = env.step([0, 0, 0, -1, 0, 0, 0])
    np.testing.assert_array_equal(a, b)
    np.testing.assert_array_equal(first[0], second[0])
    assert first[1:] == second[1:]
    assert a.shape == (52,)
    for seed in range(20):
        _, info = env.reset(seed=seed)
        assert info["initial_lane_clearance"] < 0.03
        assert 0.25 <= env.backend.defender.speed <= 0.55
        assert len(env.world.robots) == 3


def test_defender_moves_and_is_speed_bounded():
    env = DefendedEnv()
    env.reset(seed=7)
    before = env.world.robot(DEFENDER)
    for _ in range(10):
        c = env.backend.defender.command(env.world)
        assert math.hypot(c.vx, c.vy) <= env.backend.defender.speed + 1e-9
        env.world = env.backend.step({env.passer: Command(dribbler=1)})
    after = env.world.robot(DEFENDER)
    assert (after.x, after.y) != (before.x, before.y)
    assert env.backend.distance_moved > 0


def test_interception_detected_between_control_frames(monkeypatch):
    env = DefendedEnv()
    env.reset(seed=1)
    original = env.world
    d = original.robot(DEFENDER)
    touched = replace(original, ball=Ball(d.x + 0.10, d.y))
    env.backend.base.last_kick_events = []
    env.backend.base.last_substep_worlds = [original, touched, original]
    monkeypatch.setattr(env.backend.base, "step", lambda _: original)
    _, _, done, _, info = env.step([0, 0, 0, -1, 0, 0, 0])
    assert done and not info["is_success"] and info["reason"] == "defender_interception"
    assert info["defender_ball_contacts"] == 1


def test_defender_robot_collision_rejected(monkeypatch):
    env = DefendedEnv()
    env.reset(seed=1)
    original = env.world
    p = original.robot(env.passer)
    touched = replace(
        original,
        robots=tuple(
            replace(r, x=p.x + 0.1, y=p.y) if r.key == DEFENDER else r for r in original.robots
        ),
    )
    env.backend.base.last_kick_events = []
    env.backend.base.last_substep_worlds = [touched]
    monkeypatch.setattr(env.backend.base, "step", lambda _: touched)
    _, _, done, _, info = env.step([0, 0, 0, -1, 0, 0, 0])
    assert done and info["reason"] == "defender_collision"


def test_defender_remains_active_after_scoring_handoff():
    env = DefendedEnv()
    env.reset(seed=3)
    env.state.received = True
    shot = scoring_handoff(env)
    assert shot.backend is env.backend and shot.world is env.world
    checks = env.backend.checks
    shot.step([0, 0, 0, -1])
    assert env.backend.checks > checks and len(shot.world.robots) == 3


def test_observation_sees_defender_and_rejects_nonfinite():
    env = DefendedEnv()
    a, _ = env.reset(seed=2)
    changed = replace(
        env.world,
        robots=tuple(replace(r, y=r.y + 0.5) if r.key == DEFENDER else r for r in env.world.robots),
    )
    b = defended_observation(changed, env.passer, env.receiver, env.state, env.cfg)
    np.testing.assert_array_equal(a[:36], b[:36])
    assert not np.array_equal(a[36:], b[36:])
    changed = replace(
        changed,
        robots=tuple(
            replace(r, x=float("nan")) if r.key == DEFENDER else r for r in changed.robots
        ),
    )
    with pytest.raises(ValueError):
        defended_observation(changed, env.passer, env.receiver, env.state, env.cfg)
