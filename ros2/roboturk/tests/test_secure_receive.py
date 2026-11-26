from dataclasses import replace
import math

import pytest

from roboturk.model import Ball, Command, Config, Robot, RobotKey, World
from roboturk.secure_receive import ReceiveProfile, SecureTransfer
from roboturk.team import TransferState


@pytest.mark.parametrize("heading", [0.0, 1.8, -2.5])
def test_receiver_meets_ball_on_release_axis_and_masks_kick(heading):
    passer, receiver = RobotKey("blue", 1), RobotKey("blue", 0)
    dx, dy = math.cos(heading), math.sin(heading)
    w = World(
        1.0,
        (Robot(passer, 0, 0, heading), Robot(receiver, 2 * dx, 2 * dy, heading + math.pi)),
        Ball(0.5 * dx, 0.5 * dy, dx, dy),
    )
    arm = SecureTransfer.__new__(SecureTransfer)
    arm.profile = ReceiveProfile(0.6, 0.9, 4.0)
    state = TransferState()
    raw = [0, 0, 0, -1, 0, 0, 0]

    def command(w):
        return arm.decode_action(raw, w, passer, receiver, state, Config())

    assert command(w)[receiver] == Command(dribbler=1)
    state.passed = True
    cmd = command(w)
    assert cmd[passer] == Command()
    assert cmd[receiver].vx == pytest.approx(0.6)
    assert cmd[receiver].vy == pytest.approx(0)
    assert cmd[receiver].kick == 0
    at_mouth = replace(w, ball=Ball(1.885 * dx, 1.885 * dy, dx, dy))
    assert command(at_mouth)[receiver].vx == pytest.approx(-0.9)
