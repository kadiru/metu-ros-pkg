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
    assert a.shape == (56,)
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


def test_defended_contract_matches_opposite_attacking_direction():
    env = DefendedEnv()
    a, _ = env.reset(seed=4)
    w = env.world
    rotated = replace(
        w,
        attack_sign=-w.attack_sign,
        robots=tuple(
            replace(r, x=-r.x, y=-r.y, yaw=r.yaw + math.pi, vx=-r.vx, vy=-r.vy) for r in w.robots
        ),
        ball=replace(w.ball, x=-w.ball.x, y=-w.ball.y, vx=-w.ball.vx, vy=-w.ball.vy),
    )
    b = defended_observation(rotated, env.passer, env.receiver, env.state, env.cfg)
    np.testing.assert_allclose(a, b, atol=1e-6)


def test_defender_reacts_to_ball_crossing_direction():
    from roboturk.model import world_vector

    env = DefendedEnv()
    env.reset(seed=8)
    d = env.world.robot(DEFENDER)
    for lateral in (-1, 1):
        w = replace(env.world, ball=Ball(d.x - 1, d.y, 1, lateral))
        c = env.backend.defender.command(w)
        _, vy = world_vector(c.vx, c.vy, d.yaw)
        assert lateral * vy > 0.2


def test_receiver_action_frames_and_offline_upgrade_agree():
    from roboturk.defended import upgrade_actions
    from roboturk.team_contract import team_action
    from roboturk.model import body_vector, world_vector

    env = DefendedEnv()
    for seed in range(12):
        obs, _ = env.reset(seed=seed)
        r = env.world.robot(env.receiver)
        sign = env.world.direction(env.receiver.team)
        vx, vy = body_vector(0.3, -0.7, r.yaw)
        commands = {env.passer: Command(dribbler=1), env.receiver: Command(vx, vy, 0.2, 1)}
        old = team_action(commands, env.passer, env.receiver, env.cfg)
        converted = upgrade_actions(obs[:52], old)
        np.testing.assert_allclose(converted, env.encode_commands(commands), atol=1e-7)
        assert converted[4] == pytest.approx(sign * 0.3 / env.cfg.max_speed)
        result = env.decode_action(converted)[env.receiver]
        np.testing.assert_allclose(
            world_vector(result.vx, result.vy, r.yaw), [0.3, -0.7], atol=1e-7
        )


def test_attack_frame_velocity_features_match_world_measurements():
    env = DefendedEnv()
    for seed in range(8):
        env.reset(seed=seed)
        w = replace(
            env.world,
            ball=replace(env.world.ball, vx=0.6, vy=-0.2),
            robots=tuple(
                replace(r, vx=-0.4, vy=0.3) if r.key == env.receiver else r
                for r in env.world.robots
            ),
        )
        obs = defended_observation(w, env.passer, env.receiver, env.state, env.cfg)
        np.testing.assert_allclose(
            obs[52:],
            w.attack_sign * np.array([0.6, -0.2, -0.4, 0.3]) / env.cfg.max_speed,
            atol=1e-7,
        )
