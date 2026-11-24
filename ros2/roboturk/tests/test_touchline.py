from dataclasses import replace
import math

import pytest

from roboturk import field
from roboturk.five_roles import CFG
from roboturk.model import Ball, Command, Referee, Robot, RobotKey, World
from roboturk.open_motion import Steering, steer_team
from roboturk.three_match import filter_commands
from roboturk.touchline import END_LIMIT, SIDE_LIMIT, brake_axis, recovery_command


@pytest.mark.parametrize("side", [-1, 1])
def test_crossing_only_for_live_recovery(side):
    r = Robot(RobotKey("blue", 4), 1.3, side * 2.98)
    ball = Ball(1, side * (CFG.ball_exit_y - 0.0001))
    w = World(0, (r,), ball, Referee.RUNNING)
    assert recovery_command(w, r, "Chaser", None) is not None
    assert recovery_command(w, r, "Cover", None) is None
    for invalid in (
        replace(ball, y=side * (CFG.ball_exit_y + 0.0001)),
        replace(ball, x=CFG.ball_exit_x + 0.001),
        replace(ball, visible=False),
    ):
        assert recovery_command(replace(w, ball=invalid), r, "Chaser", None) is None
    assert recovery_command(w, r, "Chaser", RobotKey("yellow", 1)) is None


@pytest.mark.parametrize("side", [-1, 1])
@pytest.mark.parametrize("learned", [True, False])
def test_robot_can_leave_and_brake_in_both_steering_paths(side, learned):
    actor = Steering(0.38, 0.30, 0.60)

    def output(y, active):
        r = Robot(RobotKey("blue", 1), END_LIMIT, side * y)
        cmd = {r.key: Command(0.5, side * 1.0)}
        permit = {r.key} if active else set()
        if learned:
            return steer_team((r,), "blue", cmd, {r.key: (0, 0)}, actor, CFG.dt, permit)[r.key]
        return filter_commands(World(0, (r,), Ball(0, side * 3), Referee.RUNNING), cmd, permit)[0][
            r.key
        ]

    assert side * output(2.95, True).vy > 0
    assert output(2.95, False).vy == 0
    assert output(SIDE_LIMIT, True).vy == 0
    assert output(2.95, True).vx == 0
    assert SIDE_LIMIT + CFG.robot_radius < field.WIDTH / 2 + field.WALL_MARGIN
    assert END_LIMIT + CFG.robot_radius < field.LENGTH / 2


def test_braking_reserves_reaction_tick_on_each_axis():
    for gap in (0, 0.001, 0.03, 0.10):
        v = brake_axis(SIDE_LIMIT - gap, 3, SIDE_LIMIT, CFG.dt)
        a = CFG.max_acceleration / math.sqrt(2)
        assert v * CFG.dt + v * v / (2 * a) <= gap + 1e-12
    assert brake_axis(SIDE_LIMIT + 0.01, -0.6, SIDE_LIMIT, CFG.dt) == -0.6
