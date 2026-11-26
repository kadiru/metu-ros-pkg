import math

import pytest

pytest.importorskip("torch")

from roboturk.carrier_motor import protected_motion, track_roller
from roboturk.model import Ball, Command, Robot, RobotKey, World

MOTOR = dict(drive=0.32, step_v=0.19, grip=17, pivot=4.2, cap=1.16)
ACTOR = Robot(RobotKey("blue", 0), 0, 0)
MATE = Robot(RobotKey("blue", 1), 1, 0.8)


def test_centered_pivot_keeps_the_mouth_at_ball_velocity():
    world = World(0, (ACTOR,), Ball(0.115, 0, 0.40, 0.20))
    cmd = track_roller(world, ACTOR, (0.40, 0.20), 2, MOTOR)
    assert cmd.vx == pytest.approx(0.40)
    assert cmd.vy + 0.115 * cmd.omega == pytest.approx(0.20)


def test_fast_diagonal_ball_respects_body_and_pivot_limits():
    world = World(0, (ACTOR,), Ball(0.125, 0.032, 1.5, -1.0))
    cmd = track_roller(world, ACTOR, (2, -1.5), 7, MOTOR)
    assert math.hypot(cmd.vx, cmd.vy) <= MOTOR["cap"] + 1e-12
    assert abs(cmd.omega) <= MOTOR["pivot"]


def test_settled_ball_returns_to_the_synchronized_pass_command():
    rival = Robot(RobotKey("yellow", 1), 1.5, -0.40)
    w = World(0, (ACTOR, MATE, rival), Ball(0.115, 0))
    p = MOTOR | dict(
        jaw_buffer=2.35,
        linger=0.15,
        near=0.42,
        side_at=0.032,
        speed_out=1.1,
        fade=0.52,
        settled=0.65,
        retreat=1.0,
    )
    default = Command(0.1, 0, 1)
    chosen, action, *_ = protected_motion(w, ACTOR, rival, MATE, default, "shield", p, False)
    assert chosen is default and action == "release"
