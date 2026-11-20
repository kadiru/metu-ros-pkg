"""Synthetic structural checks; native performance is evaluated separately."""

from dataclasses import replace
import math
import numpy as np
import pytest
from roboturk.continuous import ContinuousMatch, KEYS, Memory, observation, decode, encode
from roboturk.model import Ball, Command, Config, Referee, Robot, World


def world(ball=Ball(0.115, 0)):
    return World(
        0,
        (
            Robot(KEYS[0], 0, 0),
            Robot(KEYS[1], -2, 1),
            Robot(KEYS[2], 0.23, 0, math.pi),
            Robot(KEYS[3], 2, -1),
        ),
        ball,
        Referee.RUNNING,
    )


class Replay:
    name = "synthetic_replay"

    def __init__(self):
        self.resets = 0

    def reset(self, w):
        self.world = w
        self.resets += 1
        return w

    def step(self, commands):
        self.world = replace(self.world, time=self.world.time + 0.02)
        self.last_substep_worlds = [self.world] * 4
        self.last_kick_events = []
        return self.world


def test_contested_ball_has_no_exclusive_owner():
    w = world()
    m = Memory()
    m.reset(w)
    for _ in range(40):
        w = replace(w, time=w.time + 0.02)
        m.observe([w] * 4, [], Config())
    assert all(v == 0 for v in m.dwell.values())


def test_rotation_and_team_swap_preserve_observation():
    w = world(Ball(0.8, 0.5))
    m = Memory()
    m.reset(w)
    a = observation(w, KEYS[0], m)
    rotated = replace(
        w,
        attack_sign=-1,
        ball=Ball(-0.8, -0.5),
        robots=tuple(replace(r, x=-r.x, y=-r.y, yaw=r.yaw + math.pi) for r in w.robots),
    )
    mm = Memory()
    mm.reset(rotated)
    np.testing.assert_allclose(a, observation(rotated, KEYS[0], mm), atol=1e-7)
    mapping = {KEYS[0]: KEYS[2], KEYS[1]: KEYS[3], KEYS[2]: KEYS[0], KEYS[3]: KEYS[1]}
    swapped = replace(
        w, attack_sign=-1, robots=tuple(replace(r, key=mapping[r.key]) for r in w.robots)
    )
    mm.reset(swapped)
    np.testing.assert_allclose(a, observation(swapped, KEYS[2], mm), atol=1e-7)


def test_contact_and_turnover_continue_without_reset():
    b = Replay()
    e = ContinuousMatch(b, duration=2)
    e.reset(1)
    actions = np.stack([encode(0, Command())] * 4)
    # Stable exclusive blue possession, then yellow possession; only test data moves the ball.
    e.world = b.world = world(Ball(-0.115, 0))
    e.world = b.world = replace(
        e.world,
        robots=tuple(replace(r, yaw=math.pi) if r.key == KEYS[0] else r for r in e.world.robots),
    )
    e.memory.reset(e.world)
    for _ in range(20):
        e.step(actions)
    assert e.last_owner == KEYS[0]
    e.world = b.world = replace(
        e.world,
        ball=Ball(0.345, 0),
        robots=tuple(replace(r, yaw=0) if r.key == KEYS[2] else r for r in e.world.robots),
    )
    for _ in range(20):
        e.step(actions)
    assert e.last_owner == KEYS[2]
    assert e.turnovers["yellow"] == 1
    assert not e.finished and b.resets == 1
    assert e.calls["blue0"] == e.steps
    assert e.physics_checks == 4 * e.steps


def test_pending_restart_cannot_mutate_before_fresh_inference():
    b = Replay()
    e = ContinuousMatch(b)
    e.reset(1)
    e.pending_restart = "goal"
    with pytest.raises(RuntimeError):
        e.step(np.zeros((4, 35), np.float32))
    assert b.resets == 1 and e.pending_restart == "goal"
    e.restart_if_needed()
    assert b.resets == 2 and e.events[-1]["type"] == "referee_restart"


def test_referee_halt_and_skill_kick_masks():
    w = world()
    for mode in range(7):
        a = encode(mode, Command(vx=10, vy=10, kick=3.5))
        _, c = decode(a, w, KEYS[0])
        assert math.hypot(c.vx, c.vy) <= 1.200001
        assert (c.kick > 0) == (mode in (2, 3))
        assert decode(a, replace(w, referee=Referee.HALT), KEYS[0])[1] == Command()


def test_behavior_selects_a_distinct_motion_head():
    w = world()
    a = np.zeros(35, np.float32)
    a[0] = 1
    a[7] = 0.5
    a[7 + 4 * 6 + 1] = 0.5
    first = decode(a, w, KEYS[0])[1]
    a[0] = 0
    a[6] = 1
    second = decode(a, w, KEYS[0])[1]
    assert first.vx == 0.6 and first.vy == 0
    assert second.vx == 0 and second.vy == 0.6
