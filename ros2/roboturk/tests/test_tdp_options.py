import math
from dataclasses import replace
import numpy as np
from roboturk.backend import ReferenceBackend
from roboturk.model import Ball, Referee, Robot, RobotKey, World
from roboturk.tdp_options import BallOptionEnv, BallOptionTeacher
from roboturk.tdp_playbook import TdpPlaybook, ReceptionContract


def test_option_contract_finite_and_referee_halt():
    for kind in ("receive", "redirect", "clear"):
        env = BallOptionEnv(ReferenceBackend(), kind)
        for seed in (5000000, 5000001, 5000002):
            obs, _ = env.reset(seed=seed)
            assert obs.shape == (34,) and np.isfinite(obs).all()
            action = BallOptionTeacher().action(env)
            env.world = replace(env.world, referee=Referee.HALT)
            command = env.command(action)
            assert command.vx == command.vy == command.omega == command.kick == 0


def test_initial_intent_is_frame_and_color_invariant():
    env = BallOptionEnv(ReferenceBackend(), "receive")
    env.reset(seed=5000000)
    before = env.observation()
    w = env.world
    env.world = replace(
        w,
        attack_sign=-w.attack_sign,
        robots=tuple(
            replace(r, x=-r.x, y=-r.y, yaw=r.yaw + math.pi, vx=-r.vx, vy=-r.vy) for r in w.robots
        ),
        ball=replace(w.ball, x=-w.ball.x, y=-w.ball.y, vx=-w.ball.vx, vy=-w.ball.vy),
    )
    env.sign = -env.sign
    i = env.intent
    env.intent = replace(
        i,
        point=(-i.point[0], -i.point[1]),
        heading=i.heading + math.pi,
        outlet=(-i.outlet[0], -i.outlet[1]),
    )
    np.testing.assert_allclose(before, env.observation(), atol=1e-6)


def test_commitment_holds_and_referee_cancels():
    a, b = RobotKey("blue", 0), RobotKey("blue", 1)
    w = World(0, (Robot(a, 1, 0), Robot(b, 2.5, 1)), Ball(2.4, 1), Referee.RUNNING)
    plan = ReceptionContract("p1", (2.4, 1), (1.115, 0), 1, 3)
    book = TdpPlaybook()
    first = book.choose(w, "blue", [a], plan)
    assert first.intent.kind == "redirect"
    altered = replace(plan, identifier="p2", point=(0, 2))
    assert book.choose(replace(w, time=0.1), "blue", [a], altered) == first
    assert book.choose(replace(w, referee=Referee.HALT), "blue", [a], plan) is None
    assert book.events[-1]["event"] == "cancel"


def test_blocked_redirect_stops_and_impossible_arrival_rejected():
    a = RobotKey("blue", 0)
    w = World(
        0, (Robot(a, 1, 0), Robot(RobotKey("yellow", 0), 3, 0)), Ball(2.4, 1), Referee.RUNNING
    )
    plan = ReceptionContract("p", (2.4, 1), (1.115, 0), 2, 4)
    assert TdpPlaybook().choose(w, "blue", [a], plan).intent.kind == "receive"
    assert (
        TdpPlaybook().choose(w, "blue", [a], replace(plan, point=(-2, 2), arrival_at=0.01)) is None
    )


def test_role_assignment_ignores_input_order_and_expiration_cancels():
    a, b = RobotKey("blue", 0), RobotKey("blue", 1)
    w = World(
        0,
        (Robot(b, -3.5, 0), Robot(a, -3.1, 0), Robot(RobotKey("yellow", 0), -1.6, -0.4)),
        Ball(-3, 0),
        Referee.RUNNING,
    )
    first = TdpPlaybook().choose(w, "blue", [b, a])
    second = TdpPlaybook().choose(w, "blue", [a, b])
    assert first == second and first.intent.kind == "clear"
    assert first.intent.outlet[1] > 0
    book = TdpPlaybook()
    book.choose(w, "blue", [a, b])
    assert book.choose(replace(w, time=9), "blue", [a, b]) is None
