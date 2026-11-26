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


def test_private_kickoff_cannot_read_the_opponents_goal_or_hidden_motion():
    from roboturk.ssl_vision import detection
    from roboturk.team_kickoff import IndependentKickoffs

    blue = Robot(RobotKey("blue", 0), -0.4, 0)
    yellow = Robot(RobotKey("yellow", 0), 0.1, 0.51)
    world = World(0, (blue, yellow), Ball(0, 0))
    target = replace(world, robots=(replace(blue, x=1), replace(yellow, y=-1)))
    variant = replace(target, robots=(target.robots[0], replace(yellow, x=-3, y=1.8)))
    first, second = IndependentKickoffs(target), IndependentKickoffs(variant)
    assert set(first.planners["blue"].targets) == {blue.key}
    for tick in range(5):
        measured = replace(world, robots=(blue, replace(yellow, y=yellow.y - 0.04 * tick)))
        frame = detection(measured, tick, tick * 0.04, 0.022).detection
        hidden = replace(measured, robots=(blue, replace(yellow, x=-4, y=2, yaw=1.7, vx=2, vy=-1)))
        a, _ = first.commands(measured, 0.04, frame)
        b, _ = second.commands(hidden, 0.04, frame)
        assert a[blue.key] == b[blue.key]


def test_same_opponent_position_but_different_observed_motion_changes_steering():
    from roboturk.team_kickoff import TeamKickoffPlanner

    blue = Robot(RobotKey("blue", 0), -0.4, 0)
    yellow = RobotKey("yellow", 0)

    def command_after(previous_y):
        planner = TeamKickoffPlanner((replace(blue, x=1),))
        planner.commands((blue,), {yellow: (0.1, previous_y)}, 0, 0.04)
        return planner.commands((blue,), {yellow: (0.1, 0.50)}, 0.04, 0.04)[0][blue.key]

    assert command_after(0.54) != command_after(0.46)


def test_separate_planners_exchange_ends_with_both_goals_initially_occupied():
    from roboturk.ssl_vision import detection
    from roboturk.team_kickoff import IndependentKickoffs

    a = Robot(RobotKey("blue", 0), -1, 0)
    b = Robot(RobotKey("yellow", 0), 1, 0)
    world = World(0, (a, b), Ball(0, 0))
    target = replace(world, robots=(replace(a, x=1), replace(b, x=-1)))
    drive = IndependentKickoffs(target)
    for t in range(400):
        frame = detection(world, t, world.time, 0.022)
        out, ready = drive.commands(world, 0.04, frame.detection)
        if ready:
            break
        updated = []
        for r in world.robots:
            c = out[r.key]
            vx, vy = world_vector(c.vx, c.vy, r.yaw)
            assert math.hypot(vx, vy) <= 1 + 1e-9 and c.kick == c.dribbler == 0
            updated.append(replace(r, x=r.x + vx * 0.04, y=r.y + vy * 0.04, vx=vx, vy=vy))
        a, b = updated
        assert math.hypot(a.x - b.x, a.y - b.y) >= 0.289
        world = replace(world, robots=tuple(updated), time=world.time + 0.04)
    assert ready
