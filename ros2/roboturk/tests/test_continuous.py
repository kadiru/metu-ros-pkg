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


def set_holder(e, b, key=None):
    robots = tuple(Robot(k, (-2, 0, 2, 3)[i], 1 if i == 3 else 0) for i, k in enumerate(KEYS))
    holder = next((r for r in robots if r.key == key), None)
    ball = Ball(holder.x + 0.115, holder.y) if holder else Ball(0, 2)
    e.world = b.world = World(e.steps * 0.02, robots, ball, Referee.RUNNING)
    for _ in range(20):
        e.step(np.stack([encode(0, Command())] * 4))


def test_interception_invalidates_old_pass_reception():
    b = Replay()
    e = ContinuousMatch(b, duration=10)
    e.reset(1)
    set_holder(e, b, KEYS[0])
    e.last_pass = {
        "type": "pass",
        "step": e.steps,
        "robot": "blue0",
        "team": "blue",
        "ball": [-1.885, 0],
    }
    set_holder(e, b, KEYS[2])
    assert e.last_pass is None
    set_holder(e, b, KEYS[1])
    assert e.receptions["blue"] == 0
    assert e.turnovers["blue"] == e.turnovers["yellow"] == 1


def test_same_robot_recovery_is_counted_after_a_loose_interval():
    b = Replay()
    e = ContinuousMatch(b, duration=10)
    e.reset(1)
    set_holder(e, b, KEYS[0])
    assert e.acquisitions["blue"] == 1 and e.recoveries["blue"] == 0
    set_holder(e, b)
    set_holder(e, b, KEYS[0])
    assert e.acquisitions["blue"] == 2 and e.recoveries["blue"] == 1
    set_holder(e, b, KEYS[0])
    assert e.acquisitions["blue"] == 2
    assert b.resets == 1


def test_unfinished_matches_are_not_awarded_wins_or_draws():
    from roboturk.continuous import summary

    e = ContinuousMatch(Replay())
    e.reset(1)
    e.reason = "physics_failure"
    unfinished = e.result()
    e.reason = "time"
    finished = e.result()
    s = summary([unfinished, finished])
    assert s["unfinished"] == 1 and s["completed"] == 1 and s["draws"] == 1
    assert sum(s["wins"].values()) == 0


def test_local_features_expose_body_closing_speed_and_wall_margin():
    from roboturk.continuous_features import augment

    w = world(Ball(1, 1))
    w = replace(
        w,
        robots=tuple(
            replace(r, x=4.1, yaw=math.pi / 2, vy=1.0) if r.key == KEYS[0] else r for r in w.robots
        ),
    )
    m = Memory()
    m.reset(w)
    x = observation(w, KEYS[0], m)
    assert len(x) == 122
    assert x[92] == pytest.approx(-0.5)
    assert x[93] == pytest.approx(0, abs=1e-6)
    assert x[109] == pytest.approx(0.25, abs=1e-5)
    np.testing.assert_array_equal(augment(np.stack([x[:88], x[:88]])), np.stack([x, x]))


def test_teacher_does_not_assign_both_teammates_to_cover():
    from roboturk.continuous_teacher import ContinuousTeacher, avoid

    b = Replay()
    e = ContinuousMatch(b)
    e.reset(1)
    e.world = World(
        0,
        (
            Robot(KEYS[0], -1, 0),
            Robot(KEYS[1], -1.05, 0.1),
            Robot(KEYS[2], 2, 0),
            Robot(KEYS[3], 3, 1),
        ),
        Ball(0, 0),
        Referee.RUNNING,
    )
    e.memory.reset(e.world)
    e.memory.mode.update({KEYS[0]: 5, KEYS[1]: 5})
    actions = ContinuousTeacher().actions(e)
    assert np.argmax(actions[0, :7]) == 0
    assert np.argmax(actions[1, :7]) == 5
    # Close overlapping robots receive separating training labels, even at rest.
    c = avoid(e.world, KEYS[0], Command())
    assert c.vx > 0 and c.vy < 0
