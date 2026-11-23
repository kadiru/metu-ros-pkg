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
