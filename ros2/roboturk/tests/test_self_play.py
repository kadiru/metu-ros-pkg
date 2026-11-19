from dataclasses import replace
import math
import numpy as np
import pytest
from roboturk.backend import ReferenceBackend
from roboturk.model import Config, Referee, world_vector
from roboturk.defended import LaneDefender
from roboturk.keeper import ReactiveKeeper
from roboturk.self_play import (
    MobileOpponent,
    MobilePlayEnv,
    MobileTransfer,
    MobileFinish,
    mobile_observation,
    SHAPES,
    outcomes,
)
from roboturk.self_play_match import Carrier, MatchEnv
from roboturk.team import scoring_handoff


class Probe(Carrier):
    def predict(self, obs, deterministic=True):
        assert obs.shape == self.observation_space.shape
        self.last_observation = obs.copy()
        return super().predict(obs, deterministic)


def models():
    return {role: Probe(role) for role in SHAPES}


def environment():
    cfg, m = Config(), models()
    d, k = (MobileOpponent(r, cfg, model=m[r]) for r in ("defender", "keeper"))
    env = MobilePlayEnv(cfg, ReferenceBackend(cfg), d, k)
    env.reset(seed=2200000)
    return env, m


def test_mobile_neural_motion_not_projected_onto_anchor_lines(monkeypatch):
    def forbidden(*args):
        raise AssertionError("Analytic fallback")

    monkeypatch.setattr(LaneDefender, "command", forbidden)
    monkeypatch.setattr(ReactiveKeeper, "command", forbidden)
    env, m = environment()
    for role in ("defender", "keeper"):
        c = getattr(env.backend, role)
        c.delay = 0
        c.reset_control()
        m[role].value[:] = [-0.4, 0.3, 0.2]
        c.anchor_x = c.anchor_y = 20  # Must not create an inward correction.
        command = c.command(env.world)
        vx, vy = world_vector(command.vx, command.vy, env.world.robot(c.key).yaw)
        assert env.world.attack_sign * vx < 0
        assert env.world.attack_sign * vy > 0
        assert math.hypot(vx, vy) <= c.acceleration * env.cfg.dt + 1e-7
        assert command.omega == pytest.approx(0.8)
        assert c.command(env.world) == command and c.calls == 1
        assert c.command(replace(env.world, referee=Referee.HALT)).dribbler == 0


def test_training_and_deployed_contracts_agree_through_handoff():
    env, m = environment()
    actor = MobileTransfer(cfg=env.cfg, model=m["transfer"])
    state = actor.initial_state(env.world, env.passer, env.receiver, env.cfg)
    actor.predict(env.world, env.passer, env.receiver, state, env.cfg)
    np.testing.assert_array_equal(m["transfer"].last_observation, env.observation())
    world = env.world
    env.state.received = True
    shot = scoring_handoff(env)
    assert shot.world is world and shot.backend is env.backend
    finish = MobileFinish(cfg=env.cfg, model=m["finish"])
    finish.command(shot.world, shot.key, shot.intent, env.cfg)
    np.testing.assert_array_equal(m["finish"].last_observation, shot.observation())
    assert env.backend.defender.calls == env.backend.keeper.calls == 0
    assert shot.plan.parking(world, shot.key, env.cfg)[0] == world.attack_sign * 3.15


def test_mobile_observation_rotation_and_visible_neighbors():
    env, _ = environment()
    w, c = env.world, env.backend.defender
    a = mobile_observation(w, c)
    rotated = replace(
        w,
        attack_sign=-w.attack_sign,
        robots=tuple(
            replace(r, x=-r.x, y=-r.y, yaw=r.yaw + math.pi, vx=-r.vx, vy=-r.vy) for r in w.robots
        ),
        ball=replace(w.ball, x=-w.ball.x, y=-w.ball.y, vx=-w.ball.vx, vy=-w.ball.vy),
    )
    c.anchor_x, c.anchor_y, c.phase = -c.anchor_x, -c.anchor_y, c.phase + math.pi
    b = mobile_observation(rotated, c)
    np.testing.assert_allclose(a, b, atol=1e-6)


def test_training_uses_reserved_separate_scenarios_and_immutable_opponents():
    m = models()
    env = MatchEnv(ReferenceBackend(), Config(), "defender", m, [m], seed=42)
    obs, _ = env.reset(seed=7)
    first = env.episode_seed
    assert 2100000 <= first < 2200000 and obs.shape == (45,)
    assert env.env.backend.defender.model is env.carrier
    assert env.env.backend.keeper.model is m["keeper"]
    env.step(np.array([0.1, -0.1, 0], np.float32))
    assert np.array_equal(m["defender"].value, np.zeros(3))
    env.reset(seed=7)
    assert env.episode_seed == first


def test_defensive_contacts_are_outcomes_collisions_are_not_wins():
    result = {
        "episodes": [
            {"reason": r}
            for r in [
                "goal",
                "keeper_save",
                "defender_interception",
                "defender_collision",
                "timeout",
            ]
        ]
    }
    summary = outcomes(result)
    assert summary["goals"] == 1 and summary["defensive_stops"] == 2
    assert summary["clean_outcomes"] == 3 and summary["collisions"] == 1


def test_excursion_metrics_and_reset_measure_both_dimensions():
    env, m = environment()
    for r in ("defender", "keeper"):
        c = getattr(env.backend, r)
        c.delay = 0
        c.reset_control()
        m[r].value[:] = [0.5, 0.5, 0]
    for _ in range(10):
        env.backend.step({})
    ev = env.backend.evidence()
    assert ev["keeper_x_excursion"] > 0 and ev["defender_x_excursion"] > 0
    assert ev["keeper_y_excursion"] > 0 and ev["defender_y_excursion"] > 0
    assert ev["keeper_policy_calls"] == ev["defender_policy_calls"] == 10
    env.reset(seed=2200000)
    assert env.backend.evidence()["keeper_x_excursion"] == 0


def test_mobile_pass_speed_shared_by_environment_and_deployed_adapter():
    env, m = environment()
    actor = MobileTransfer(cfg=env.cfg, model=m["transfer"])
    action = np.zeros(7, np.float32)
    action[3] = 1
    trained = env.decode_action(action)
    deployed = actor.decode_action(action, env.world, env.passer, env.receiver, env.state, env.cfg)
    assert trained == deployed and trained[env.passer].kick == 2.5


def test_feint_target_changes_once_per_frame_and_stops_after_pass():
    from roboturk.self_play import update_receiving_target
    from roboturk.defended import DEFENDER

    env, _ = environment()
    w = replace(env.world, time=3)
    r = w.robot(env.receiver)
    b = w.ball
    guard = w.robot(DEFENDER)
    w = replace(
        w,
        robots=tuple(
            replace(guard, x=(b.x + r.x) / 2, y=(b.y + r.y) / 2) if p.key == DEFENDER else p
            for p in w.robots
        ),
    )
    env.state.receiver_target = (r.x, r.y)
    update_receiving_target(w, env.passer, env.receiver, env.state, env.cfg)
    target = env.state.receiver_target
    assert target != (r.x, r.y) and env.state.mobile_feints == 1
    update_receiving_target(w, env.passer, env.receiver, env.state, env.cfg)
    assert env.state.receiver_target == target and env.state.mobile_feints == 1
    env.state.passed = True
    update_receiving_target(replace(w, time=6), env.passer, env.receiver, env.state, env.cfg)
    assert env.state.receiver_target == target
