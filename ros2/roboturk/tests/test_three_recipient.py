"""ID and field symmetries must not change which physical teammate is selected."""

from dataclasses import replace
import math

import numpy as np
from roboturk.model import Ball, Referee, Robot, RobotKey, World
from roboturk.three_recipient import choose, train_update


def test_selection_tracks_lane_when_team_ids_and_ends_change():
    def robot(team, index, x, y):
        return Robot(RobotKey(team, index), x, y)

    carrier = robot("blue", 1, 0, 0)
    left = robot("blue", 0, 2, 1)
    right = robot("blue", 2, 2, -1)
    opponents = (robot("yellow", 2, 1, 0.5), robot("yellow", 0, 4, 0), robot("yellow", 1, 3, 1))
    w = World(0, (left, right, carrier, *opponents), Ball(0.115, 0), Referee.RUNNING)
    weights = np.array([1.0, 0, 0, 0, 0, 0])
    assert choose(w, carrier, weights).receiver == 2
    rotated = tuple(
        replace(r, x=-r.x, y=-r.y, yaw=r.yaw + math.pi, key=RobotKey(r.key.team, 2 - r.key.id))
        for r in reversed(w.robots)
    )
    w2 = replace(w, robots=rotated, ball=Ball(-0.115, 0), attack_sign=-1)
    assert choose(w2, w2.robot(carrier.key), weights).receiver == 0


def test_self_play_gradient_rewards_intended_completion():
    rows = [[0.8, 0, 0, 0, 0, 0], [0.1, 0, 0, 0, 0, 0]]

    def outcome(slot, reward):
        return dict(features=rows, slot=slot, probability=0.5, reward=reward)

    weights = train_update(np.zeros(6), [outcome(0, 1), outcome(1, -1)])
    assert np.array(rows[0]) @ weights > np.array(rows[1]) @ weights
    assert not np.any(weights[1:])
