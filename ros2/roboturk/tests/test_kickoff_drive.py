import math
from dataclasses import replace

import pytest

from roboturk.kickoff_drive import KickoffDrive, ParallelKickoffDrive, SPEED, TOLERANCE
from roboturk.model import Ball, Robot, RobotKey, World, world_vector


@pytest.mark.parametrize("planner", [KickoffDrive, ParallelKickoffDrive])
def test_swap_occupied_kickoff_positions_by_driving_round_the_other_robot(planner):
    blue = Robot(RobotKey("blue", 0), -1, 0)
    yellow = Robot(RobotKey("yellow", 0), 1, 0)
    world = World(0, (blue, yellow), Ball(0, 0))
    target = replace(world, robots=(replace(blue, x=1), replace(yellow, x=-1)))
    drive = planner(target)
    dt = 0.04
    for _ in range(650):
        out, ready = drive.commands(world, dt)
        if ready:
            break
        robots = []
        for r in world.robots:
            cmd = out[r.key]
            vx, vy = world_vector(cmd.vx, cmd.vy, r.yaw)
            assert math.hypot(vx, vy) <= SPEED + 1e-9 and cmd.kick == cmd.dribbler == 0
            robots.append(replace(r, x=r.x + dt * vx, y=r.y + dt * vy, vx=vx, vy=vy))
        assert math.dist((robots[0].x, robots[0].y), (robots[1].x, robots[1].y)) > 0.28
        world = replace(world, robots=tuple(robots), time=world.time + dt)
    assert ready
    assert abs(world.robot(blue.key).x - 1) <= TOLERANCE
    assert abs(world.robot(yellow.key).x + 1) <= TOLERANCE


def test_admit_crossing_paths_when_arrival_times_clear_the_crossing():
    a = Robot(RobotKey("blue", 0), -2, 0)
    b = Robot(RobotKey("yellow", 0), 0, -0.6)
    world = World(0, (a, b), Ball(0, 0))
    target = replace(world, robots=(replace(a, x=2), replace(b, y=0.6)))
    old, _ = KickoffDrive(target).commands(world, 0.04)
    new, _ = ParallelKickoffDrive(target).commands(world, 0.04)
    assert sum(abs(c.vx) + abs(c.vy) > 0 for c in old.values()) == 1
    assert sum(abs(c.vx) + abs(c.vy) > 0 for c in new.values()) == 2


def test_crossing_at_the_same_time_waits_until_the_route_clears():
    a = Robot(RobotKey("blue", 0), -1, 0)
    b = Robot(RobotKey("yellow", 0), 0, -1)
    world = World(0, (a, b), Ball(0, 0))
    target = replace(world, robots=(replace(a, x=1), replace(b, y=1)))
    drive = ParallelKickoffDrive(target)
    moving = []
    for _ in range(100):
        out, ready = drive.commands(world, 0.04)
        if ready:
            break
        robots = []
        for r in world.robots:
            c = out[r.key]
            robots.append(replace(r, x=r.x + 0.04 * c.vx, y=r.y + 0.04 * c.vy, vx=c.vx, vy=c.vy))
        moving.append(sum(math.hypot(r.vx, r.vy) > 0.2 for r in robots))
        assert math.dist((robots[0].x, robots[0].y), (robots[1].x, robots[1].y)) >= 0.30
        world = replace(world, robots=tuple(robots), time=world.time + 0.04)
    assert ready and moving[0] == 1 and 2 in moving
