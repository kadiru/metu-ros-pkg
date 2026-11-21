from dataclasses import replace
import math
import pytest
from roboturk.model import Ball, Config, Robot, RobotKey, World
from roboturk.decision_rewards import boundary_cost, training_settings


def test_boundary_cost_is_symmetric_and_does_not_change_world():
    key = RobotKey("yellow", 1)
    cfg = Config()
    center = World(0, (Robot(key, 0, 0),), Ball(1, 0))
    edge = replace(center, robots=(Robot(key, 4.32, 0),))
    outside = replace(center, robots=(Robot(key, 4.42, 0),))
    opposite = replace(center, robots=(Robot(key, -4.32, 0),))
    assert boundary_cost(center, key, cfg) == 0
    assert 0 < boundary_cost(edge, key, cfg) < boundary_cost(outside, key, cfg)
    assert boundary_cost(edge, key, cfg) == boundary_cost(opposite, key, cfg)
    assert edge.robot(key).x == 4.32


def test_manager_explores_choices_more_than_coordinates():
    s = training_settings("manager")
    assert math.exp(s["choice_log_std"]) > 30 * math.exp(s["target_log_std"])
    with pytest.raises(ValueError):
        training_settings("unknown")
