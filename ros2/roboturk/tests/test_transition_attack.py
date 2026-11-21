import math
import pytest
from roboturk.model import Ball, Robot, RobotKey, World

pytest.importorskip("torch", reason="The native team loads the frozen neural pass motor.")
from roboturk.transition_attack import trap, window


def test_first_touch_keeps_mouth_velocity_when_ball_is_moving_sideways():
    actor = Robot(RobotKey("blue", 0), 0, 0)
    w = World(0, (actor,), Ball(0.115, 0, 0, 0.6))
    cmd = trap(w, actor, 0, (0, 0.6), dict(tracking=0.45, centering=14.0, turn=4.0))
    assert cmd.vy == pytest.approx(0.6)
    assert cmd.kick == 0 and cmd.dribbler == 1


@pytest.mark.parametrize("sign", [-1, 1])
def test_shot_window_keeps_an_obstructing_body_out_of_the_shot(sign):
    shooter = Robot(RobotKey("blue", 0), 0, 0, math.pi if sign < 0 else 0)
    defender = Robot(RobotKey("yellow", 1), sign * 2.5, 0.07)
    w = World(0, (shooter, defender), Ball(sign * 0.115, 0), attack_sign=sign)
    x, y, gap = window(w, shooter)
    assert x * sign > 4.5 and -0.4785 < y < 0 and gap > 0.15
