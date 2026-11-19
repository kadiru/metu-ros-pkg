from dataclasses import replace
import math
import numpy as np
import pytest
from roboturk.backend import ReferenceBackend
from roboturk.model import Ball, Config
from roboturk.keeper import (
    KEEPER,
    KeeperBackend,
    KeeperFinishEnv,
    KeeperPlayEnv,
    FinishPlan,
    finish_observation,
    FinishTeacher,
)
from roboturk.team import scoring_handoff
from roboturk.defense_micro import DefenseMicroEnv, DefenseTeacher


def test_keeper_reset_and_unmodified_transfer_scenario():
    from roboturk.defended import DefendedEnv

    a, b = KeeperPlayEnv(), DefendedEnv()
    for seed in range(10):
        oa, _ = a.reset(seed=seed)
        ob, _ = b.reset(seed=seed)
        np.testing.assert_array_equal(oa, ob)
        assert len(a.world.robots) == 4
        assert 0.65 <= a.backend.keeper.speed <= 0.95
        assert 2 <= a.backend.keeper.acceleration <= 3
        assert 4 <= a.backend.keeper.delay <= 7


def test_keeper_detects_subframe_contact(monkeypatch):
    env = KeeperPlayEnv()
    env.reset(seed=1)
    original = env.world
    k = original.robot(KEEPER)
    touched = replace(original, ball=Ball(k.x, k.y))
    base = env.backend.base
    base.last_substep_worlds = [original, touched, original]
    monkeypatch.setattr(base, "step", lambda _: original)
    env.backend.step({})
    assert env.backend.failure == "keeper_save"
    assert env.backend.contacts == 1


def test_keeper_remains_active_at_handoff_without_reset():
    env = KeeperPlayEnv()
    env.reset(seed=2)
    env.state.received = True
    shot = scoring_handoff(env)
    assert isinstance(shot, KeeperFinishEnv)
    assert shot.world is env.world and shot.backend is env.backend
    assert shot.observation().shape == (32,)
    checks = env.backend.checks
    shot.step([0, 0, 0, -1])
    assert env.backend.checks > checks


def test_finish_contract_rotates_with_attack_direction():
    cfg = Config()
    env = KeeperFinishEnv(cfg, KeeperBackend(ReferenceBackend(cfg), cfg))
    env.reset(seed=3)
    w = env.world
    rotated = replace(
        w,
        attack_sign=-w.attack_sign,
        robots=tuple(
            replace(r, x=-r.x, y=-r.y, yaw=r.yaw + math.pi, vx=-r.vx, vy=-r.vy) for r in w.robots
        ),
        ball=replace(w.ball, x=-w.ball.x, y=-w.ball.y, vx=-w.ball.vx, vy=-w.ball.vy),
    )
    p = FinishPlan(rotated, env.key, cfg)
    np.testing.assert_allclose(
        env.observation(), finish_observation(rotated, env.key, p, cfg), atol=1e-6
    )
    bad = replace(w, ball=replace(w.ball, x=float("nan")))
    with pytest.raises(ValueError):
        finish_observation(bad, env.key, env.plan, cfg)


def test_tactical_aim_change_does_not_manufacture_carry():
    cfg = Config()
    env = KeeperFinishEnv(cfg, KeeperBackend(ReferenceBackend(cfg), cfg))
    env.reset(seed=4)
    distance = env.target_distance(env.world)
    env.plan.phase = 1
    env.plan.target = (env.plan.target[0], -env.plan.target[1])
    assert env.target_distance(env.world) == distance
    assert env.capture_distance - distance == 0


def test_environment_and_deployed_teacher_share_tactic_state():
    cfg = Config()
    env = KeeperFinishEnv(cfg, KeeperBackend(ReferenceBackend(cfg), cfg))
    env.reset(seed=5)
    teacher = FinishTeacher()
    from roboturk.shoot_contract import shoot_action

    expected = teacher.action(env)
    actual = shoot_action(teacher.command(env.world, env.key, env.intent, cfg), cfg)
    np.testing.assert_array_equal(expected, actual)


@pytest.mark.parametrize("kind", ["block", "cover"])
def test_defense_contract_replay_and_shape(kind):
    env = DefenseMicroEnv(kind=kind)
    teacher = DefenseTeacher()
    first, _ = env.reset(seed=3)
    step = env.step(teacher.action(env))
    second, _ = env.reset(seed=3)
    replay = env.step(teacher.action(env))
    assert first.shape == (16,)
    np.testing.assert_array_equal(first, second)
    np.testing.assert_array_equal(step[0], replay[0])
    assert step[1:] == replay[1:]
    with pytest.raises(ValueError):
        env.step([float("nan")] * env.action_space.shape[0])


def test_defense_deployment_codec_matches_environment():
    from roboturk.defense_micro import GUARD, defense_observation, defense_command
    from roboturk.model import world_vector

    for kind in ("block", "cover"):
        env = DefenseMicroEnv(kind=kind)
        for seed in range(5):
            obs, _ = env.reset(seed=seed)
            np.testing.assert_array_equal(
                obs, defense_observation(env.world, GUARD, env.target(), 0, kind, env.cfg)
            )
            action = DefenseTeacher().action(env)
            command = defense_command(action, env.world, GUARD, kind)
            vx, vy = world_vector(command.vx, command.vy, env.world.robot(GUARD).yaw)
            if kind == "block":
                assert vx == pytest.approx(0, abs=1e-9)
                assert vy == pytest.approx(env.world.attack_sign * action[0] * 1.2)
            assert command.kick == 0 and command.dribbler == 1


def test_latched_finish_phase_and_play_reset():
    from roboturk.team_play import PassReceiveScorePlay
    from roboturk.defended import tactical_state

    class Transfer:
        initial_state = staticmethod(tactical_state)

    teacher = FinishTeacher()
    teacher.plan = object()
    env = KeeperPlayEnv()
    env.reset(seed=7)
    play = PassReceiveScorePlay(Transfer(), teacher)
    play.reset(env.world, env.passer, env.receiver)
    assert teacher.plan is None
    p = FinishPlan(env.world, env.receiver, env.cfg)
    p.phase = 1
    target = p.target
    p.update(replace(env.world, time=1), env.receiver, env.cfg)
    assert p.phase == 1 and p.target == target


def test_keeper_ball_line_is_symmetric():
    from roboturk.keeper import ReactiveKeeper
    from roboturk.model import world_vector

    env = KeeperPlayEnv()
    env.reset(seed=9)
    w = replace(env.world, ball=Ball(0, 0.3))
    opposite = replace(
        w,
        attack_sign=-w.attack_sign,
        robots=tuple(replace(r, x=-r.x, y=-r.y, yaw=r.yaw + math.pi) for r in w.robots),
        ball=Ball(0, -0.3),
    )
    a, b = ReactiveKeeper(env.cfg), ReactiveKeeper(env.cfg)
    for k in (a, b):
        k.style = "ball_line"
        k.targets.clear()
    ca, cb = a.command(w), b.command(opposite)
    va = world_vector(ca.vx, ca.vy, w.robot(KEEPER).yaw)
    vb = world_vector(cb.vx, cb.vy, opposite.robot(KEEPER).yaw)
    np.testing.assert_allclose(va, -np.array(vb), atol=1e-8)
