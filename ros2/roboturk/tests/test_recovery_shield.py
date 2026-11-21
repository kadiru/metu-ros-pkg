import math
import pytest

from roboturk.model import Ball, Robot, RobotKey, World

pytest.importorskip("torch")
from roboturk.recovery_shield import shot_target


@pytest.mark.parametrize("direction", [-1, 1])
def test_shot_leads_a_crossing_defender_and_stays_inside_posts(direction):
    actor = Robot(RobotKey("blue", 0), 0, 0, yaw=0 if direction == 1 else math.pi)
    defender = Robot(RobotKey("yellow", 1), direction * 2, -0.15, vy=0.6)
    world = World(0, (actor, defender), Ball(0.1 * direction, 0), attack_sign=direction)
    gx, gy = shot_target(world, actor)
    assert gx * direction > 4.5
    assert -0.4785 < gy < 0
