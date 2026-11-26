import math
import pytest

pytest.importorskip("torch")

from roboturk.model import Robot, RobotKey, Ball, World
from roboturk.touch_escape import escape_command

CONFIG = dict(
    escape_horizon=0.12,
    escape_range=0.35,
    escape_buffer=2.1,
    escape_front=0.8,
    escape_radial=0.4,
    escape_speed=1.2,
    escape_follow=0.7,
    escape_turn=5,
    escape_drive=0.8,
    escape_step=0.52,
    escape_pivot=5.2,
    cap=1.16,
    grip=17,
)


def test_far_opponent_leaves_inherited_passing_motor_in_control():
    actor = Robot(RobotKey("blue", 0), 0, 0)
    rival = Robot(RobotKey("yellow", 2), 2.0, 0)
    assert escape_command(World(0, (actor, rival), Ball(0.115, 0)), actor, rival, CONFIG) is None


def test_escape_velocity_and_pivot_mirror_across_the_field():
    actor = Robot(RobotKey("blue", 0), 0, 0)

    def command(sign):
        rival = Robot(RobotKey("yellow", 2), 0.22, sign * 0.07, yaw=sign * math.pi, vy=-sign * 0.2)
        world = World(0, (actor, rival), Ball(0.115, 0))
        answer = escape_command(world, actor, rival, CONFIG)
        assert answer is not None
        return answer[0]

    left, right = command(1), command(-1)
    assert left.vx == pytest.approx(right.vx)
    assert left.vy == pytest.approx(-right.vy)
    assert left.omega == pytest.approx(-right.omega)


def test_contact_forecast_flags_an_opponent_inside_the_roller():
    from roboturk.model import Command
    from roboturk.touch_contact import forecast

    actor = Robot(RobotKey("blue", 2), 0, 0)
    ball = Ball(0.115, 0)
    commands = [Command(0, 0, 0, 1)]
    p = dict(
        contact_steps=4,
        body_contact=0.110,
        impulse=1.0,
        own_edge=0.75,
        rival_edge=1.6,
        retouch_weight=2.0,
    )

    def risk(x):
        marker = Robot(RobotKey("yellow", 0), x, 0, yaw=math.pi)
        return forecast(World(0, (actor, marker), ball), actor, marker, commands, p)[0]

    assert risk(0.235) > risk(0.70)


def test_swing_releases_to_passing_after_the_rival_falls_behind():
    from roboturk.touch_contact import swing_roller

    actor = Robot(RobotKey("blue", 2), 0, 0)
    ball = Ball(0.115, 0)
    marker = Robot(RobotKey("yellow", 1), -0.19, 0.015)
    assert (
        swing_roller(
            World(0, (actor, marker), ball), actor, marker, dict(swing_range=0.4, swing_clear=0.035)
        )
        is None
    )
