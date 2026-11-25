import pytest

from roboturk.five_roles import FiveRoles, kickoff
from roboturk.five_strategy import BALANCED, decode_strategies, describe_play
from roboturk.model import RobotKey


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
