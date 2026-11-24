from dataclasses import replace

import pytest

from roboturk.model import Command, Robot, RobotKey
from roboturk.open_motion import Steering, observed_velocity, steer_team


def test_opponent_input_is_causal_position_history_only():
    blue = Robot(RobotKey("blue", 2), 0, 0)
    yellow = Robot(RobotKey("yellow", 3), 0.50, 0, vx=10)
    robots = (blue, yellow)
    measured = observed_velocity(robots, {yellow.key: (0.54, 0)}, 0.04)
    assert measured[yellow.key] == pytest.approx((-1, 0))
    cmd = Command(1, 0, 0.5, 1, 2.6)
    req = {blue.key: cmd}
    actor = Steering(0.38, 0.30, 0.60)

    def steer(roster, requests):
        return steer_team(roster, "blue", requests, measured, actor, 0.04)[blue.key]

    sent = steer(robots, req)
    assert sent != cmd and (sent.omega, sent.dribbler, sent.kick) == (0.5, 1, 2.6)
    assert sent == steer((blue, replace(yellow, vx=-10, vy=123, yaw=2)), req)
    with pytest.raises(ValueError):
        steer(robots, {**req, yellow.key: Command(-1, 0, 0)})
    assert all(v == (0, 0) for v in observed_velocity(robots, {}, 0.04).values())


def test_keeper_mouth_access_requires_clearance_from_the_posts():
    keeper = Robot(RobotKey("blue", 0), -4.35, 0.0)
    policy = Steering(0.48, 0.305, 0.66)
    request = {keeper.key: Command(-0.4, 0, 0, 1)}

    def drive(robot, active):
        return steer_team(
            (robot,),
            "blue",
            request,
            {robot.key: (0, 0)},
            policy,
            0.04,
            goal_cover={robot.key: -1} if active else None,
        )[robot.key]

    assert drive(keeper, True).vx < 0
    assert drive(keeper, False).vx == 0
    assert drive(replace(keeper, y=0.40), True).vx == 0
    assert drive(replace(keeper, y=0.32, vy=0.6), True).vx == 0


def test_handed_off_keeper_can_approach_quickly_and_still_brakes_at_roller_range():
    keeper = Robot(RobotKey("blue", 0), 0, 0)
    presser = Robot(RobotKey("blue", 1), 0, 1)
    rival = Robot(RobotKey("yellow", 1), 0.76, 0)
    policy = Steering(0.48, 0.305, 0.66)
    observed = {r.key: (0, 0) for r in (keeper, presser, rival)}
    commands = {keeper.key: Command(1.16, 0, 0, 1), presser.key: Command()}

    def approach(other, timed):
        return steer_team(
            (keeper, presser, other),
            "blue",
            commands,
            observed,
            policy,
            0.04,
            challenges={keeper.key: other},
            keeper_handoffs={keeper.key: presser.key} if timed else None,
        )[keeper.key].vx

    assert approach(rival, timed=False) == pytest.approx(0.32)
    assert approach(rival, timed=True) > 0.9
    assert approach(replace(rival, x=0.202), timed=True) < 0.13
