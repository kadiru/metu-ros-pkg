from dataclasses import replace
import math

import pytest

from roboturk.five_roles import Assignment, CFG
from roboturk.model import Command, Robot, RobotKey
from roboturk.timed_motion import ACCEL, TIMES, Leg, axis_to, choose, clear, plan_team


@pytest.mark.parametrize("target", [-2, -0.03, 0, 0.03, 2])
@pytest.mark.parametrize("incoming", [-1.4, -0.55, 0, 0.55, 1.4])
def test_axis_arrives_at_rest_even_if_it_must_reverse(target, incoming):
    axis = axis_to(0, incoming, target)
    assert all(t >= 0 and abs(a) <= ACCEL for t, a in axis.phases)
    assert axis.at(0) == pytest.approx((0, incoming))
    assert axis.at(axis.duration + 1) == pytest.approx((target, 0), abs=1e-12)


def test_planar_acceleration_and_speed_budgets():
    leg = Leg.to((0.2, 1.3), (0.38, -0.52), (-2.0, 0.3))
    for i in range(math.ceil(leg.duration / CFG.dt)):
        v, n = leg.velocity(i * CFG.dt), leg.velocity((i + 1) * CFG.dt)
        assert math.dist(v, n) <= CFG.max_acceleration * CFG.dt + 1e-12
        assert math.hypot(*n) <= 1.2 + 1e-12
    assert leg.position(leg.duration) == pytest.approx((-2, 0.3))


def test_crossing_depends_on_arrival_time():
    robot = Robot(RobotKey("blue", 1), 0, 0)
    target = (1.2, 0)

    def track(speed):
        return [(0.55, -0.55 + t * speed) for t in TIMES]

    v, path, mode = choose(robot, target, [track(1)], 0.25)
    assert mode == "detour" and v[1] != 0 and clear(path, [track(1)], 0.25)
    assert choose(robot, target, [track(-1)], 0.25)[2] == "direct"
    assert choose(robot, target, [[(0, 0)] * len(TIMES)], 0.25)[2] == "brake"


def test_swept_separation_and_static_obstacles():
    assert not clear([(-0.3, 0), (0.3, 0)], [[(0.3, 0), (-0.3, 0)]], 0.2)
    assert not clear([(3, 0), (4, 0)], [], 0.2)
    assert not clear([(0, 2.8), (0, 2.92)], [], 0.2)
    assert clear([(3, 1.4), (4, 1.4)], [], 0.2)


def test_teammate_reservation_and_opponent_information_boundary():
    a = Robot(RobotKey("blue", 2), -0.55, 0)
    b = Robot(RobotKey("blue", 1), 0, -0.55)
    q = Robot(RobotKey("yellow", 3), 2.5, -1.4)
    targets = {a.key: (0.65, 0), b.key: (0, 0.65)}
    roles = {
        a.key: Assignment("Marker", targets[a.key]),
        b.key: Assignment("Cover", targets[b.key]),
    }
    cmd = {r.key: Command(omega=1.2, dribbler=1) for r in (a, b)}

    def plan(opponent):
        counts = {}
        out = plan_team(
            (a, b, opponent), "blue", cmd, targets, roles, {q.key: (0, 0)}, 0.25, counts
        )
        return out, counts

    out, counts = plan(q)
    assert counts == {"direct": 1, "detour": 1}
    assert out[a.key].vy == 0 and out[b.key].vx != 0
    assert out[b.key].omega == 1.2 and out[b.key].dribbler == 1
    assert plan(replace(q, vx=7, vy=-5, yaw=1.3)) == (out, counts)
