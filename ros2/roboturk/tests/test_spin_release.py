from dataclasses import replace
import math

import pytest

from roboturk.five_play import turn_release
from roboturk.model import Ball, Config, Referee, Robot, RobotKey, World, wrap
from roboturk.spin_release import MAX_DRIBBLE, SpinRelease, arc_clear, turn_rate

CFG = Config(dt=0.04)
MARGIN = 0.314


def opening(sign):
    yaw = sign * 2.2
    radius = CFG.possession_x
    actor = Robot(RobotKey("blue", 1), -radius * math.cos(yaw), -radius * math.sin(yaw), yaw)
    blocker = Robot(RobotKey("yellow", 1), -0.4 * math.cos(yaw / 2), -0.4 * math.sin(yaw / 2))
    return actor, blocker, World(0, (actor, blocker), Ball(0, 0), Referee.RUNNING)


@pytest.mark.parametrize("sign", (-1, 1))
def test_spin_takes_the_long_clear_arc_and_keeps_its_direction_across_pi(sign):
    actor, blocker, world = opening(sign)
    spin = SpinRelease()
    spin.observe(world, (actor.key,), actor.key)
    assert not arc_clear(world, actor, -actor.yaw, turn_rate(CFG), CFG, {}, MARGIN)
    turn = spin.start(world, actor, (2, 0), None, CFG, {}, MARGIN)
    assert turn.reverse and math.pi < turn.sweep * sign < math.tau

    for tick in range(1, 12):
        yaw = wrap(actor.yaw + turn.sweep * tick / 11)
        r = replace(
            actor, yaw=yaw, x=-CFG.possession_x * math.cos(yaw), y=-CFG.possession_x * math.sin(yaw)
        )
        frame = replace(world, time=tick * CFG.dt, robots=(r, blocker))
        command, ready = spin.step(frame, r, turn.target, CFG, {}, MARGIN)
        if tick < 11:
            assert sign * command.omega > 0 and not ready
        else:
            assert ready and abs(command.omega) < 1e-8
    assert turn.turned == pytest.approx(turn.sweep)


def test_spin_cancels_on_a_new_shot_block_and_refuses_an_overlong_dribble():
    actor, blocker, world = opening(1)
    spin = SpinRelease()
    spin.observe(world, (actor.key,), actor.key)
    spin.grip_origin = (-MAX_DRIBBLE, 0)
    assert spin.start(world, actor, (2, 0), None, CFG, {}, MARGIN) is None
    spin.grip_origin = (0, 0)
    assert spin.start(world, actor, (2, 0), None, CFG, {}, MARGIN)
    blocked = replace(world, robots=(actor, replace(blocker, x=1, y=0)))
    command, flight, task = turn_release(
        blocked, actor, spin, (2, 0), None, CFG, {}, MARGIN, None, 4.0, True
    )
    assert not command.kick and flight is None and spin.turn is None
    assert task == "Hold the ball after a blocked turn"
