from dataclasses import replace
import math
import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env
from roboturk.model import Ball, Command, Robot
from roboturk.team import PassReceiveEnv, scoring_handoff
from roboturk.team_contract import team_commands, team_observation


def test_team_replay_and_gym_contract():
    env = PassReceiveEnv()
    check_env(env, skip_render_check=True)
    first, _ = env.reset(seed=31)
    a = env.step([0, 0, 0, -1, 0, 0, 0])
    second, _ = env.reset(seed=31)
    b = env.step([0, 0, 0, -1, 0, 0, 0])
    np.testing.assert_array_equal(first, second)
    np.testing.assert_array_equal(a[0], b[0])
    assert a[1:] == b[1:]
    assert first.shape == (36,)


def test_team_actions_and_role_masks():
    env = PassReceiveEnv()
    env.reset(seed=0)
    action = [0.1, 0.2, 0.3, 1, 0.2, 0.1, 0.3]
    c = team_commands(action, env.passer, env.receiver, False, env.cfg)
    assert c[env.passer].kick == 1.8 and c[env.passer].dribbler == 0
    assert c[env.receiver].kick == 0 and c[env.receiver].dribbler == 1
    c = team_commands(action, env.passer, env.receiver, True, env.cfg)
    assert c[env.passer] == Command()
    with pytest.raises(ValueError):
        team_commands([float("nan")] * 7, env.passer, env.receiver, False, env.cfg)


def test_team_observation_rotation_invariance():
    env = PassReceiveEnv()
    env.reset(seed=4)
    w = env.world
    rotated = replace(
        w,
        robots=tuple(
            replace(r, x=-r.y, y=r.x, yaw=r.yaw + math.pi / 2, vx=-r.vy, vy=r.vx) for r in w.robots
        ),
        ball=replace(w.ball, x=-w.ball.y, y=w.ball.x, vx=-w.ball.vy, vy=w.ball.vx),
    )
    np.testing.assert_allclose(
        env.observation(),
        team_observation(rotated, env.passer, env.receiver, env.state, env.cfg),
        atol=1e-6,
    )


def test_early_pass_fails_even_if_kick_applies():
    env = PassReceiveEnv()
    env.reset(seed=0)
    _, _, done, _, info = env.step([0, 0, 0, 1, 0, 0, 0])
    assert done and info["reason"] == "invalid_pass" and info["applied_passes"] == 1
    with pytest.raises(ValueError):
        scoring_handoff(env)


def test_receiver_cannot_bypass_actual_pass(monkeypatch):
    env = PassReceiveEnv()
    env.reset(seed=0)
    r = env.world.robot(env.receiver)
    current = replace(
        env.world, ball=Ball(r.x + 0.115 * math.cos(r.yaw), r.y + 0.115 * math.sin(r.yaw))
    )
    monkeypatch.setattr(env.backend, "step", lambda _: current)
    env.backend.last_substep_worlds = [current]
    env.backend.last_kick_events = []
    _, _, done, _, info = env.step([0, 0, 0, -1, 0, 0, 0])
    assert done and not info["received"] and not info["is_success"]


def test_pass_loss_between_control_samples_fails(monkeypatch):
    env = PassReceiveEnv()
    env.reset(seed=0)
    held = env.world
    lost = replace(held, ball=replace(held.ball, x=held.ball.x + 0.3))
    env.backend.last_substep_worlds = [held, lost, held]
    env.backend.last_kick_events = []
    monkeypatch.setattr(env.backend, "step", lambda _: held)
    _, _, done, _, info = env.step([0, 0, 0, -1, 0, 0, 0])
    assert done and info["lost_checks"] == 1 and info["reason"] == "passer_lost_possession"


def test_handoff_keeps_physics_state_and_requires_carry():
    env = PassReceiveEnv()
    env.reset(seed=2)
    env.state.received = True
    shot = scoring_handoff(env)
    assert shot.backend is env.backend and shot.world is env.world and shot.key == env.receiver
    assert shot.acquired and shot.carried_progress == 0 and not shot.shot
    assert shot.minimum_carry == 0.5 and shot.capture_distance == shot.target_distance(env.world)


def test_collision_during_transfer_fails(monkeypatch):
    env = PassReceiveEnv()
    env.reset(seed=0)
    p = env.world.robot(env.passer)
    current = replace(env.world, robots=(p, Robot(env.receiver, p.x + 0.1, p.y)))
    monkeypatch.setattr(env.backend, "step", lambda _: current)
    env.backend.last_substep_worlds = [current]
    env.backend.last_kick_events = []
    _, _, done, _, info = env.step([0, 0, 0, -1, 0, 0, 0])
    assert done and info["reason"] == "robot_collision"


def test_play_memory_matches_transfer_monitor_and_handles_stops():
    from roboturk.team import TransferTeacher
    from roboturk.team_play import PassReceiveScorePlay
    from roboturk.model import Referee

    env = PassReceiveEnv()
    env.reset(seed=1)
    play = PassReceiveScorePlay(None, None)
    play.reset(env.world, env.passer, env.receiver)
    teacher = TransferTeacher()
    for _ in range(800):
        _, _, done, truncated, _ = env.step(teacher.action(env))
        play.observe([env.world], env.backend.last_kick_events)
        assert (
            play.state.passed,
            play.state.received,
            play.state.passer_dwell,
            play.state.receiver_dwell,
        ) == (
            env.state.passed,
            env.state.received,
            env.state.passer_dwell,
            env.state.receiver_dwell,
        )
        if done or truncated:
            break
    assert play.state.received
    assert all(
        c == Command() for c in play.commands(replace(env.world, referee=Referee.HALT)).values()
    )
    with pytest.raises(ValueError):
        play.observe([env.world], [])
