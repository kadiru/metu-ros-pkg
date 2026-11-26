import pytest

pytest.importorskip("torch")

from roboturk.model import Ball, Command, Robot, RobotKey, World
from roboturk.roller_rescue import rescue_motion
from roboturk.touch_context import first_touch_features, select_motor


def test_rescue_preserves_a_settled_ball_and_waits_for_sole_control():
    r = Robot(RobotKey("blue", 1), 0, 0)
    enemy = Robot(RobotKey("yellow", 2), 0.6, 0.2)
    w = World(0, (r, enemy), Ball(0.115, 0.0))
    p = dict(
        rescue_look=0.085,
        rescue_age=12,
        rescue_edge=0.75,
        retouch_gate=1.15,
        guard_age=14,
        rescue_side=0.45,
        rescue_drive=0.35,
        rescue_step=0.245,
        rescue_grip=18,
        rescue_turn=5,
        cap=1.16,
    )
    assert rescue_motion(w, r, enemy, Command(), p, 40) is None
    assert rescue_motion(w, r, enemy, Command(0, 1.1), p, 6) is None
    cmd, mode, edge, _ = rescue_motion(w, r, enemy, Command(0, 1.1), p, 20)
    assert mode == "shield" and edge > 1 and abs(cmd.vy) < 1.1


def test_selector_uses_relative_motion_and_is_independent_of_team_and_id():
    def inputs(team, ids):
        other = "yellow" if team == "blue" else "blue"
        r = Robot(RobotKey(team, ids[0]), 0.1, 0.2, 0.7, 0.05, 0.10)
        m = Robot(RobotKey(team, ids[1]), 0.7, -0.65)
        m2 = Robot(RobotKey(team, ids[2]), 1.2, 1.1)
        guard = Robot(RobotKey(other, ids[1]), 0.31, 0.21, 3, 0.25, 0.41)
        return first_touch_features(World(0, (m, guard, r, m2), Ball(0.19, 0.29, 0.42, -0.13)), r)

    x = inputs("blue", [1, 2, 0])
    assert x == pytest.approx(inputs("yellow", [2, 0, 1]))
    tree = dict(feature=0, threshold=0.5, left=dict(action=1), right=dict(action=3))
    assert select_motor(tree, x) == 1
    assert select_motor(tree, [0.8] + x[1:]) == 3
