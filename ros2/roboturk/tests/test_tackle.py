from dataclasses import replace
import math
from types import SimpleNamespace

import numpy as np

from roboturk.five_play import open_play
from roboturk.five_roles import CFG, FiveRoles, kickoff
from roboturk.model import Ball, Command, RobotKey
from roboturk.open_motion import Steering

CARRIER = RobotKey("blue", 1)
PRESSER = RobotKey("yellow", 1)


def contest(distance):
    w = kickoff(12, "blue", 1)
    robots = []
    for r in w.robots:
        if r.key == CARRIER:
            robots.append(replace(r, x=0, y=0, yaw=0))
        elif r.key == PRESSER:
            robots.append(replace(r, x=distance, y=0, yaw=math.pi))
        else:
            robots.append(replace(r, x=-2 if r.key.team == "blue" else 2, y=r.key.id * 0.6 - 1.3))
    return replace(w, robots=tuple(robots), ball=Ball(CFG.possession_x, 0))


def test_presser_tackle_only_acts_on_a_live_opponent_carrier():
    w = contest(0.6)
    roles = FiveRoles()
    for _ in range(3):
        roles.observe(w)
    seen = []

    def act(world, presser, carrier, observed, has_ball):
        seen.append((presser.key, carrier.key, has_ball))
        return Command(dribbler=1)

    measured = {r.key: (0, 0) for r in w.robots}
    tackle = SimpleNamespace(command=act)
    policy = Steering(0.48, 0.305, 0.66)
    # The carrier's actual pass selector remains part of the normal entry point.
    weights = {team: np.zeros(6) for team in ("blue", "yellow")}
    open_play(w, roles, weights, measured, policy, touchline=False, tackle=tackle)
    assert seen == [(PRESSER, CARRIER, False)]
    seen.clear()
    loose = replace(w, ball=Ball(-1, -1))
    open_play(loose, roles, weights, measured, policy, touchline=False, tackle=tackle)
    assert not seen


def test_double_grip_grants_one_bounded_window_for_a_tackle():
    w = contest(0.23)
    standard = FiveRoles()
    fitted = FiveRoles()
    for tactics in (standard, fitted):
        tactics.control = CARRIER
        tactics.last_touch = 0
    for _ in range(4):
        standard.observe(w)
        fitted.observe(w, tackling=PRESSER)
    assert standard.contain is not None and fitted.contain is None
    for _ in range(16):
        fitted.observe(w, tackling=PRESSER)
    assert fitted.contain is not None
