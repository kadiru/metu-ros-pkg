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


def test_physical_goal_requires_blocked_lane_open_receiver_and_unbroken_recovery():
    import numpy as np
    from roboturk.blocked_audit import opening_result

    states = np.zeros((360, 22))
    robots = states[:, 4:].reshape(-1, 6, 3)
    robots[:, :, :2] = [-2, 2]
    robots[:, 0] = [-0.115, 0, 0]
    robots[:, 1] = [1.115, 1, np.pi]
    robots[:, 4] = [1.0, 0, 0]
    robots[:24, 3] = [0.115, 0, np.pi]
    states[81:131, :2] = np.linspace([0.02, 0.02], [1, 1], 50)
    states[131:169, :2] = [1, 1]
    states[169:, :2] = np.linspace([1.02, 1], [4.54, 0], 191)
    game = dict(
        seed=2,
        seconds=3.6,
        options=[
            dict(team="blue", recovery=43, tick=12, actor=0, action="pass", receiver=1, until=80)
        ],
        passes=[
            dict(
                team="blue",
                passer=0,
                receiver=1,
                slot=0,
                tick=20,
                end=39,
                ball=[0, 0],
                result="complete",
            )
        ],
        events=[
            dict(event="kick", team="blue", robot=0, speed=2.2, tick=20),
            dict(event="kick", team="blue", robot=1, speed=4, tick=42),
            dict(event="goal", team="yellow", tick=89),
        ],
    )
    protocol = dict(
        blocked_center_gap_m=0.24, minimum_pass_lane_gap_m=0.19, minimum_receiver_space_m=0.48
    )
    assert opening_result(states, game, protocol)["qualified"]
    robots[:, 4, 1] = -1.2
    assert opening_result(states, game, protocol)["outcome"] == "dribble lane clear"
    robots[:, 4, 1] = 0
    robots[47, 5, :2] = [1.115, 1.2]
    assert opening_result(states, game, protocol)["outcome"] == "receiver covered"
    robots[62, 3] = [0.115, 0, np.pi]
    assert opening_result(states, game, protocol)["outcome"] == "opponent retouch before pass"
