import math
from dataclasses import replace

from roboturk.kickoff_drive import KickoffDrive, SPEED, TOLERANCE
from roboturk.model import Ball, Robot, RobotKey, World, world_vector


def test_swap_occupied_kickoff_positions_by_driving_round_the_other_robot():
    blue = Robot(RobotKey("blue", 0), -1, 0)
    yellow = Robot(RobotKey("yellow", 0), 1, 0)
    world = World(0, (blue, yellow), Ball(0, 0))
    target = replace(world, robots=(replace(blue, x=1), replace(yellow, x=-1)))
    drive = KickoffDrive(target)
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
