from dataclasses import replace
import pytest

pytest.importorskip("torch", reason="3v3 native strategy loads the learned pass motor.")
from roboturk.blocked_outlet import geometry, select
from roboturk.model import Ball, Robot, RobotKey, World


def test_blocker_behind_ball_does_not_close_forward_dribble_lane():
    direct, lane, space = geometry((0, 0), 1, (0, 1), [(-0.20, 0), (0.9, 0.18), (0.1, 1.0)])
    assert direct == pytest.approx(0.18)
    assert lane == pytest.approx(0.10)
    assert space == pytest.approx(0.10)
    assert geometry((0, 0), 1, (0, 1), [(-0.20, 0)])[0] > 1


@pytest.mark.parametrize("open_id", [1, 2])
def test_lane_geometry_selects_either_identity_and_switches_on_clear_path(open_id):
    actor = Robot(RobotKey("blue", 0), -0.115, 0)
    free = Robot(RobotKey("blue", open_id), 1.0, 1.2)
    marked = Robot(RobotKey("blue", 3 - open_id), 1.0, -1.2)
    central = Robot(RobotKey("yellow", 0), 0.85, 0)
    marker = Robot(RobotKey("yellow", 1), 1.0, -1.05)
    rival = Robot(RobotKey("yellow", 2), -0.5, -0.6)
    w = World(0, (actor, free, marked, central, marker, rival), Ball(0, 0))
    p = dict(receiver=[2.4, 0.5, 0.75, 0.2, -0.2], bias=0.77, block=2.6, lane=0.85, finish=1)
    choice, action, _ = select(w, actor, p)
    assert (choice.receiver, action) == (open_id, "pass")
    w = replace(
        w, robots=(actor, free, marked, replace(central, y=2.0), replace(marker, x=-0.3), rival)
    )
    assert select(w, actor, p)[1] == "shoot"
