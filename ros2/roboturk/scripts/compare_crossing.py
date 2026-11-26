"""Reproduce a two-teammate crossing with the analytic planar drive model."""

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path

from roboturk.five_roles import Assignment, CFG
from roboturk.model import Robot, RobotKey, drive_velocity
from roboturk.open_motion import Steering, observed_velocity, steer_team
from roboturk.three_recipient import move
from roboturk.timed_motion import plan_team


def crossing(actor, timed):
    robots = (Robot(RobotKey("blue", 2), -0.90, 0), Robot(RobotKey("blue", 1), 0, -0.90))
    marker, cover = (r.key for r in robots)
    targets = {marker: (0.90, 0), cover: (0, 0.90)}
    roles = {
        marker: Assignment("Marker", targets[marker]),
        cover: Assignment("Cover", targets[cover]),
    }
    previous, counts = {}, {}
    samples = [[(r.x, r.y) for r in robots]]
    clearance = math.inf
    finished = None
    substeps = 20
    dt = CFG.dt / substeps
    for n in range(180):
        measured = observed_velocity(robots, previous, CFG.dt)
        previous = {r.key: (r.x, r.y) for r in robots}
        commands = {r.key: move(r, *targets[r.key], 0, 1.2) for r in robots}
        commands = steer_team(robots, "blue", commands, measured, actor, CFG.dt)
        if timed:
            commands = plan_team(
                robots, "blue", commands, targets, roles, measured, actor.clearance, counts
            )
        for _ in range(substeps):
            moved = []
            for r in robots:
                vx, vy = drive_velocity(r, commands[r.key], dt, CFG)
                moved.append(replace(r, x=r.x + vx * dt, y=r.y + vy * dt, vx=vx, vy=vy))
            robots = tuple(moved)
            a, b = robots
            clearance = min(clearance, math.hypot(a.x - b.x, a.y - b.y) - 2 * CFG.robot_radius)
        samples.append([(r.x, r.y) for r in robots])
        if all(math.dist((r.x, r.y), targets[r.key]) < 0.03 for r in robots):
            finished = (n + 1) * CFG.dt
            break
    return dict(samples=samples, finish=finished, gap=clearance, counts=counts)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    actor = Steering(**json.loads((root / "policies/open_motion_v1/actor.json").read_text()))
    data = {
        name: crossing(actor, timed) for name, timed in (("reactive", False), ("trajectory", True))
    }
    args.output.write_text(json.dumps(data, indent=2) + "\n")
