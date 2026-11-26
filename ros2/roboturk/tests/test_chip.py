from dataclasses import replace
import math

import pytest

from roboturk.backend import ReferenceBackend
from roboturk.chip import chip_pass
from roboturk.model import Ball, Command, Config, Referee, Robot, RobotKey, World, possession
from roboturk.role_telemetry import KickTrace, action_events


def test_chip_pass_needs_a_flat_lane_blocker_it_can_clear():
    cfg = Config(max_kick=6.5)
    carrier = Robot(RobotKey("blue", 1), -cfg.possession_x, 0)
    receiver = Robot(RobotKey("blue", 2), 2, 0)
    defender = Robot(RobotKey("yellow", 1), 0.7, 0)
    world = World(0, (carrier, receiver, defender), Ball(0, 0), Referee.RUNNING)

    def plan(blocker=defender, limits=cfg):
        return chip_pass(
            replace(world, robots=(carrier, receiver, blocker)),
            carrier.key,
            receiver.key,
            (receiver.x, receiver.y),
            limits,
        )

    chip = plan()
    assert chip is not None
    assert chip.landing == pytest.approx((1.45, 0))
    assert chip.speed < 6.5
    assert plan(replace(defender, y=0.3)) is None
    assert plan(replace(defender, x=0.2)) is None
    assert plan(limits=replace(cfg, max_kick=3)) is None


def test_chip_cap_includes_vertical_speed_and_airborne_ball_cannot_be_gripped():
    cfg = Config(max_kick=6.5)
    robot = Robot(RobotKey("blue", 1), 0, 0)
    backend = ReferenceBackend(cfg)
    initial = World(0, (robot,), Ball(cfg.possession_x, 0), Referee.RUNNING)
    backend.reset(initial)
    state = backend.step({robot.key: Command(kick=10, kick_elevation=math.pi / 4)})
    assert state.ball.vz > 4
    assert state.ball.vx == pytest.approx(6.5 / math.sqrt(2))
    assert state.ball.z > cfg.ball_radius
    above_roller = replace(state, ball=replace(state.ball, x=cfg.possession_x))
    assert not possession(above_roller, robot.key, cfg)
    assert backend.last_kick_events[0][2] == 6.5


def test_goal_line_trace_records_measured_3d_speed_and_location():
    cfg = Config()
    keys = [("blue", 1), ("blue", 2)]
    launch = {
        "tick": 1,
        "event": "pass",
        "actor": 0,
        "receiver": 1,
        "origin": [4.4, 0.2, cfg.ball_radius],
        "speed": 5.0,
        "elevation": 0.2,
    }
    trace = KickTrace(cfg)
    state = World(0, (), Ball(4.4, 0.2, 4.0, z=0.1, vz=3.0))
    events = []
    trace.start(launch, state, events)
    trace.sample(2, replace(state, time=0.04, ball=replace(state.ball, x=4.56, z=0.18)), events)
    assert events[0]["point"] == pytest.approx((cfg.ball_exit_x, 0.2, 0.16325))
    assert events[0]["speed"] == pytest.approx(5.0)
    assert events[0]["outcome"] == "goal mouth"
    timeline = list(action_events([launch, *events], keys))
    assert [item[2] for item in timeline] == ["chip", "kick_trace"]
