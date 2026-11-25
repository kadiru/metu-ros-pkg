from dataclasses import replace
import math
from types import SimpleNamespace

import numpy as np
import pytest

from roboturk.five_play import open_play
from roboturk.five_receive import FiveReceive, carrying
from roboturk.five_roles import FiveRoles, Flight, kickoff
from roboturk.model import Ball, Command, Robot, RobotKey, World, world_vector


@pytest.mark.parametrize("sign", (-1, 1))
def test_receiver_yields_to_an_incoming_ball_before_roller_contact(sign):
    key = RobotKey("blue", 1)
    receiver = Robot(key, 2.7 * sign, 0, math.pi if sign > 0 else 0)
    world = World(0, (receiver,), Ball(2.40 * sign, 0, 1.65 * sign, 0), attack_sign=sign)
    profile = FiveReceive(0.92, 6, 0.62)
    command = profile.command(world, receiver, (2.7 * sign, 0))
    vx, _ = world_vector(command.vx, command.vy, receiver.yaw)
    assert vx * sign > 0.8
    assert command.dribbler == 1 and command.kick == 0


def test_five_player_pass_reaches_the_receiver_profile():
    world = replace(kickoff(12, "blue", 1), ball=Ball(0.2, 0.1, 1.3, 0.04))
    receiver = RobotKey("blue", 2)
    tactics = FiveRoles()
    tactics.flight = Flight(receiver.team, receiver, (0.9, 1.3), 2.0)
    actor = world.robot(receiver)
    calls = []

    def receive(w, player, target):
        calls.append((w, player, target))
        return Command(dribbler=1)

    weights = {team: np.zeros(6) for team in ("blue", "yellow")}
    commands, _, roles, _ = open_play(
        world, tactics, weights, {}, None, receive=SimpleNamespace(command=receive)
    )
    assert roles[receiver].role == "Receiver"
    assert calls == [(world, actor, tactics.flight.target)]
    assert commands[receiver] == Command(dribbler=1)


def test_carrying_ignores_a_tiny_heading_error_and_keeps_the_roller_on():
    actor = Robot(RobotKey("blue", 1), 0, 0)
    world = World(0, (actor,), Ball(0.115, 0))
    command = carrying(world, actor, (0.8, 0.0), 0.02)
    assert command.omega == 0
    assert 0 < command.vx < 0.8 and command.dribbler == 1
