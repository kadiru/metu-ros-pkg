"""Regression checks for the carpet, vector limits and the shared physical pitch."""

from dataclasses import asdict, replace
import math
from pathlib import Path
import runpy

import pytest

from roboturk.backend import ReferenceBackend
from roboturk.carpet import carpet_velocity, ground_step, SWITCH_RATIO
from roboturk.capture_contract import capture_command
from roboturk.model import Ball, Command, Config, Robot, RobotKey, World, compatible_policy_config
from roboturk.model import bounded
from roboturk.ssl_vision import geometry

ROOT = Path(__file__).resolve().parents[1]
CFG = Config()


@pytest.mark.parametrize("angle", [0, 0.34, math.pi / 4, math.pi / 2, math.pi])
def test_free_roll_stops_in_finite_time_for_every_direction(angle):
    c, s = math.cos(angle), math.sin(angle)
    v = [2 * c, 2 * s, 0, 0, 0, 0]
    dt = 0.002
    distance = 0.0
    for _ in range(3000):
        new = carpet_velocity(v, dt)
        assert math.hypot(*new[:2]) <= math.hypot(*v[:2]) + 1e-10
        assert abs(new[0] * s - new[1] * c) < 1e-10
        distance += math.hypot(*new[:2]) * dt
        v = new
    expected, stop = ground_step(2, SWITCH_RATIO * 2, 8)
    assert stop == 0 and math.hypot(*v[:2]) == 0
    assert distance == pytest.approx(expected, abs=0.015)
    backend = ReferenceBackend(CFG)
    backend.reset(World(0, (), Ball(0, 0, 2 * c, 2 * s)))
    for _ in range(300):
        w = backend.step({})
    assert math.hypot(w.ball.vx, w.ball.vy) == 0
    assert math.hypot(w.ball.x, w.ball.y) == pytest.approx(expected, abs=1e-8)


def test_drive_limits_magnitude_during_diagonal_reversal_and_braking():
    key = RobotKey("blue", 3)
    backend = ReferenceBackend(CFG)
    w = backend.reset(World(0, (Robot(key, 0, 0, 0.75),), Ball(-4, -2)))
    for cmd in (Command(20, 20), Command(-20, -20), Command()):
        for _ in range(100):
            old = w.robot(key)
            w = backend.step({key: cmd})
            r = w.robot(key)
            assert math.hypot(r.vx, r.vy) <= 3 + 1e-10
            assert math.dist((r.vx, r.vy), (old.vx, old.vy)) <= 5 * CFG.dt + 1e-10
    assert (r.vx, r.vy) == (0, 0)


def test_old_specialist_keeps_its_command_scale():
    old = replace(CFG, max_speed=2, max_kick=5)
    manifest = asdict(old)
    del manifest["max_acceleration"]
    assert compatible_policy_config(manifest, CFG)
    assert not compatible_policy_config(dict(manifest, dt=0.1), CFG)
    action = [1, 1, 0, 1]
    assert capture_command(action, CFG) == capture_command(action, old)


def test_every_world_loads_the_pitch_advertised_on_ssl_vision():
    build = runpy.run_path(str(ROOT / "scripts/build_ssl_field.py"))
    assert build["PROTO"].read_text() == build["field_proto"]()
    for p in (ROOT / "src/roboturk_ros/webots/worlds").glob("*.wbt"):
        s = p.read_text()
        assert "DEF FIELD SslDivisionBField { }" in s and "basicTimeStep 2" in s, p
        assert "coulombFriction [ 0 ] rollingFriction 0 0 0" in s
    packet = geometry()
    assert packet.IsInitialized()
    f = packet.geometry.field
    assert (f.field_length, f.field_width, f.goal_width, f.goal_depth, f.goal_height) == (
        9000,
        6000,
        1000,
        180,
        160,
    )
    assert (
        f.penalty_area_width,
        f.penalty_area_depth,
        f.boundary_width,
        f.boundary_width_goal_line,
    ) == (2000, 1000, 300, 300)
    assert len(f.field_lines) == 12 and f.goal_center_to_penalty_mark == 6000
    lines = {line.name: line for line in f.field_lines}
    assert lines["LeftFieldLeftPenaltyStretch"].p1.y == -1000
    assert (lines["RightPenaltyStretch"].p1.x, lines["RightPenaltyStretch"].p2.y) == (3500, 1000)
    assert f.field_arcs[0].radius == 500 and f.field_arcs[0].thickness == 10


def test_kick_limits_apply_to_requested_launch_speed():
    request = Command(kick=99)
    assert bounded(request, replace(CFG, max_kick=20)).kick == 10
    assert bounded(request, replace(CFG, max_kick=6.5)).kick == 6.5
    assert bounded(Command(kick=2.4), CFG).kick == 2.4
