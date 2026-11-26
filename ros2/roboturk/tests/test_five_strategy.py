from dataclasses import replace

import numpy as np
import pytest

from roboturk.five_decision import CONTROLLERS
from roboturk.five_roles import FiveRoles, kickoff
from roboturk.five_strategy import BALANCED, decode_strategies, describe_play
from roboturk.model import RobotKey, Skill


def test_playbooks_change_target_geometry_and_independent_pass_triggers():
    choices = decode_strategies(
        {
            "blue": {"strategy": "wide", "pass_preference": 1},
            "yellow": {"strategy": "direct", "pass_preference": -1},
        }
    )
    world = kickoff(12, "blue", 1)
    targets = {}
    for tag, strategy in (("baseline", BALANCED), *choices.items()):
        roles = FiveRoles()
        roles.control = RobotKey("blue", 1)
        phase, assigned = roles.assign(world, "blue", strategy=strategy)
        assert phase == "Attack"
        assert assigned[roles.control].role == "Carrier"
        targets[tag] = next(item.target for item in assigned.values() if item.role == "Left wing")
    assert targets["blue"][1] > targets["baseline"][1]
    assert targets["yellow"][0] > targets["baseline"][0]
    assert choices["blue"].pass_is_available(False, 1.55, 0.2)
    assert not BALANCED.pass_is_available(False, 1.55, 0.2)
    assert not choices["yellow"].pass_is_available(False, 1.1, 0.8)
    assert choices["yellow"].book.shoot_range > BALANCED.book.shoot_range


def test_play_trace_uses_tactic_and_pass_flight_as_steps():
    task = describe_play(BALANCED, "Attack", task="Line up a pass")
    flight = describe_play(BALANCED, "Attack", in_flight=True)
    assert task["play"] == flight["play"] == "Build the attack"
    assert task["step"] == "Line up a pass"
    assert flight["step"] == "Receive the pass"
    assert describe_play(BALANCED, "Defend", in_flight=True)["step"] == "Cover the pass"


def test_practice_preferences_are_bounded():
    with pytest.raises(ValueError, match="Unknown strategy or pass preference"):
        decode_strategies(
            {
                "blue": {"strategy": "wide", "pass_preference": 2},
                "yellow": {"strategy": "direct", "pass_preference": 0},
            }
        )


def test_controller_config_is_independent_per_team_and_legacy_defaults_to_balanced():
    entry = dict(strategy="balanced", pass_preference=0)
    config = dict(blue=entry | {"controller": "priority"}, yellow=entry)
    strategies = decode_strategies(config)
    assert strategies["yellow"] == BALANCED
    assert describe_play(strategies["blue"], "Recover")["strategy"] == "Local Priority"
    with pytest.raises(ValueError, match="Unknown decision controller"):
        decode_strategies(config | {"yellow": entry | {"controller": "invalid"}})


def test_priority_fills_the_first_role_before_minimizing_remaining_travel():
    robots = kickoff(12, "blue", 1).robots[1:3]
    first, second = robots
    costs = {
        (first.key, "Chaser"): 1,
        (second.key, "Chaser"): 2,
        (first.key, "Cover"): 0,
        (second.key, "Cover"): 5,
    }
    candidates = (robots, robots[::-1])
    roles = ("Chaser", "Cover")
    assert CONTROLLERS["balanced"].lineup(candidates, roles, costs) == robots[::-1]
    assert CONTROLLERS["priority"].lineup(candidates, roles, costs) == robots


@pytest.mark.parametrize("sign", [1, -1])
def test_coordinated_switches_receiver_when_a_moving_defender_blocks_the_first_lane(sign):
    world = kickoff(12, "blue", sign)
    poses = {
        RobotKey("blue", 1): (-0.115, 0, 0),
        RobotKey("blue", 2): (2, 0, 0),
        RobotKey("blue", 3): (1.4, -1.8, 0),
        RobotKey("blue", 4): (-2.5, 0, 0),
        RobotKey("yellow", 1): (1, 0.42, -0.97),
        RobotKey("yellow", 2): (3.5, 2.2, 0),
        RobotKey("yellow", 3): (3.5, -2.2, 0),
        RobotKey("yellow", 4): (4.1, 1, 0),
    }
    robots = []
    for robot in world.robots:
        if robot.key in poses:
            x, y, vy = poses[robot.key]
            robot = replace(robot, x=sign * x, y=sign * y, vy=sign * vy)
        robots.append(robot)
    world = replace(world, robots=tuple(robots))
    carrier = world.robot(RobotKey("blue", 1))
    baseline = CONTROLLERS["balanced"].attack(world, carrier, BALANCED, np.zeros(6), False)
    coordinated = CONTROLLERS["coordinated"].attack(world, carrier, BALANCED, np.zeros(6), False)
    assert baseline.intent.receiver == RobotKey("blue", 2)
    assert coordinated.intent.skill == Skill.PASS
    assert coordinated.intent.receiver == RobotKey("blue", 3)
