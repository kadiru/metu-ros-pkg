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
    assert a.shape == (62,)
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
    state = replace(env.state, receiver_target=tuple(-v for v in env.state.receiver_target))
    b = defended_observation(rotated, env.passer, env.receiver, state, env.cfg)
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
        assert converted[4] == pytest.approx(sign * 0.3 / 2.0)
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
            obs[52:56],
            w.attack_sign * np.array([0.6, -0.2, -0.4, 0.3]) / 2.0,
            atol=1e-7,
        )


def test_receiving_point_has_turn_margin_and_stays_fixed():
    from roboturk.defended import receiving_point

    env = DefendedEnv()
    for seed in range(100):
        obs, _ = env.reset(seed=seed)
        target = env.state.receiver_target
        r, d = env.world.robot(env.receiver), env.world.robot(DEFENDER)
        assert abs(target[1]) <= env.cfg.width / 2 - 0.65
        assert (target[1] - env.world.ball.y) * (r.y - d.y) < 0
        assert target == receiving_point(env.world, env.passer, env.receiver, env.cfg)
        env.step([0, 0, 0, -1, 0.1, 0.1, 0])
        assert env.state.receiver_target == target
        assert obs.shape == (62,)


def test_tactical_target_matches_play_reset_and_legacy_contract_remains_available():
    from roboturk.defended import tactical_state, DEFENDED_CONTRACT_V2
    from roboturk.team_play import PassReceiveScorePlay

    class TargetPolicy:
        initial_state = staticmethod(tactical_state)

    env = DefendedEnv()
    env.reset(seed=12)
    play = PassReceiveScorePlay(TargetPolicy(), None)
    play.reset(env.world, env.passer, env.receiver)
    assert play.state.receiver_target == env.state.receiver_target
    np.testing.assert_array_equal(
        env.observation(),
        defended_observation(env.world, env.passer, env.receiver, play.state, env.cfg),
    )
    old = DefendedEnv(contract=DEFENDED_CONTRACT_V2)
    obs, _ = old.reset(seed=12)
    assert obs.shape == (56,) and old.state.receiver_target is None


def test_closing_marker_preserves_teammate_openings_and_reacts_to_visible_ball_motion():
    from roboturk.closing_press import ClosingPassEnv
    from roboturk.team import PassReceiveEnv

    opposed, original = ClosingPassEnv(), PassReceiveEnv()
    original.reset(seed=18)
    _, first = opposed.reset(seed=18)
    assert opposed.world.robots[:2] == original.world.robots
    assert opposed.world.ball == original.world.ball
    assert opposed.reset(seed=18)[1] == first
    marker = opposed.backend.defender
    waiting = marker.command(opposed.world)
    assert (waiting.vx, waiting.vy) == (0, 0) and marker.press_start is None
    moving = replace(opposed.world, time=1.5, ball=replace(opposed.world.ball, vx=1.4))
    command = marker.command(moving)
    assert 0 < math.hypot(command.vx, command.vy) <= marker.speed + 1e-9
    command = marker.command(replace(opposed.world, time=3.0))
    assert marker.press_start == 1.5 and math.hypot(command.vx, command.vy) > 0


def test_carry_returns_to_the_frozen_scorer_in_finishing_range():
    from roboturk.closing_press import ClosingCarry, ClosingPassEnv
    from roboturk.model import Intent, Skill

    env = ClosingPassEnv()
    env.reset(seed=7)
    r = env.world.robot(env.receiver)
    world = replace(
        env.world, ball=Ball(r.x + 0.115 * math.cos(r.yaw), r.y + 0.115 * math.sin(r.yaw))
    )

    class Scorer:
        def command(self, world, key, intent, cfg):
            return Command(kick=3.5)

    carry = ClosingCarry(Scorer(), 1.25, 0.85)
    for distance, kick in [(3.0, 0), (1.5, 3.5)]:
        intent = Intent(Skill.SHOOT, (world.ball.x + distance, world.ball.y))
        assert carry.command(world, r.key, intent, env.cfg).kick == kick
