from dataclasses import replace
import math

import pytest

from roboturk.five_play import open_play
from roboturk.five_roles import CFG, FiveRoles, Flight
from roboturk.five_strategy import BALANCED, describe_play
from roboturk.model import Ball, Referee, Robot, RobotKey, World
from roboturk.pass_intercept import intercept_pass


def pass_world():
    formations = {
        "blue": [(-4.2, 0), (-0.8, 0), (2.4, 0), (-0.5, 1.5), (-0.5, -1.5)],
        "yellow": [(4.2, 0), (1.05, 0.28), (1.4, 1.35), (1.5, -1.35), (3, 1)],
    }
    robots = tuple(
        Robot(RobotKey(team, i), x, y, 0 if team == "blue" else math.pi)
        for team, lineup in formations.items()
        for i, (x, y) in enumerate(lineup)
    )
    return World(0, robots, Ball(0, 0, 1.8, 0), Referee.RUNNING)


def test_applied_pass_changes_defensive_play_and_reassigns_one_field_player():
    world = pass_world()
    tactics = FiveRoles()
    receiver = RobotKey("blue", 2)
    tactics.flight = Flight("blue", receiver, (2.28, 0), 1.6, 2.15)
    audit, decisions = {}, {}
    commands, _, roles, phases = open_play(
        world, tactics, {}, {}, None, allocation_audit=audit, decision_audit=decisions
    )
    actor = RobotKey("yellow", 1)
    assert tactics.interception.actor == actor
    assert [key for key, assignment in roles.items() if assignment.role == "Interceptor"] == [actor]
    assert roles[receiver].role == "Receiver"
    assert roles[RobotKey("yellow", 0)].role == "Keeper"
    assert "Cover" in {a.role for key, a in roles.items() if key.team == "yellow"}
    assert commands[actor].dribbler == 1 and commands[actor].kick == 0
    assert audit[actor]["basis"] == "interceptor"
    assert "Measured ball motion" in decisions[actor]
    play = describe_play(
        BALANCED, phases["yellow"], in_flight=True, interception=tactics.interception
    )
    assert play["play"] == "Intercept the pass"

    _, _, disabled_roles, _ = open_play(world, tactics, {}, {}, None, pass_intercepts=False)
    assert tactics.interception is None
    assert all(a.role != "Interceptor" for a in disabled_roles.values())


def test_a_change_in_observed_ball_direction_cancels_the_assignment():
    world = pass_world()
    tactics = FiveRoles()
    tactics.flight = Flight("blue", RobotKey("blue", 2), (2.28, 0), 1.6, 2.15)
    open_play(world, tactics, {}, {}, None)
    assert tactics.interception is not None

    deflected = replace(world, time=0.04, ball=replace(world.ball, vx=-1.8))
    _, _, roles, phases = open_play(deflected, tactics, {}, {}, None)
    assert tactics.interception is None
    assert all(a.role != "Interceptor" for a in roles.values())
    assert describe_play(BALANCED, phases["yellow"], in_flight=True)["step"] == "Cover the pass"


@pytest.mark.parametrize(
    "referee,height,expires",
    [(Referee.STOP, 0.0215, 1), (Referee.RUNNING, 0.15, 1), (Referee.RUNNING, 0.0215, 0)],
)
def test_referee_airborne_ball_and_expired_pass_gate_the_response(referee, height, expires):
    world = pass_world()
    world = replace(world, referee=referee, ball=replace(world.ball, z=height))
    flight = Flight("blue", RobotKey("blue", 2), (2.28, 0), expires)
    assert intercept_pass(world, flight, CFG, 0.25, {}) is None
