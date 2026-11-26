"""A ball contact cannot be awarded by global team/color iteration order."""

from dataclasses import replace
import math
from roboturk.continuous import KEYS
from roboturk.continuous_backend import nearest_kicker
from roboturk.model import Ball, Command, Config, Referee, Robot, World


def test_nearest_kicker_is_invariant_to_color_and_iteration_order():
    w = World(
        0,
        (Robot(KEYS[0], -0.115, 0), Robot(KEYS[2], 0.125, 0, math.pi)),
        Ball(0, 0),
        Referee.RUNNING,
    )
    commands = {k: Command(kick=3.5) for k in (KEYS[0], KEYS[2])}
    assert nearest_kicker(w, commands, Config(), set())[0] == KEYS[0]
    assert (
        nearest_kicker(replace(w, robots=w.robots[::-1]), commands, Config(), set())[0] == KEYS[0]
    )
    mapping = {KEYS[0]: KEYS[2], KEYS[2]: KEYS[0]}
    swapped = replace(
        w,
        attack_sign=-1,
        robots=tuple(
            sorted((replace(r, key=mapping[r.key]) for r in w.robots), key=lambda r: r.key)
        ),
    )
    assert nearest_kicker(swapped, commands, Config(), set())[0] == KEYS[2]
    assert nearest_kicker(w, commands, Config(), {KEYS[0]})[0] == KEYS[2]


def test_exact_contact_ties_do_not_privilege_a_color():
    w = World(
        0,
        (Robot(KEYS[0], -0.115, 0), Robot(KEYS[2], 0.115, 0, math.pi)),
        Ball(0, 0),
        Referee.RUNNING,
    )
    commands = {k: Command(kick=3.5) for k in (KEYS[0], KEYS[2])}
    assert nearest_kicker(w, commands, Config(), set())[0] is None
    assert nearest_kicker(replace(w, robots=w.robots[::-1]), commands, Config(), set())[0] is None
