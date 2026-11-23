"""Replay each measured kickoff in both planners from the same native planar state."""

import json
from dataclasses import replace

import numpy as np

from placement_runner import boards
from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.five_roles import CFG, kickoff
from roboturk.kickoff_drive import KickoffDrive, ParallelKickoffDrive
from roboturk.model import Ball, Robot
from roboturk.three_match import snapshot


def run(sim, folder, cases):
    boards(sim)
    backend = ContinuousWebotsBackend(sim, CFG)
    traces = {}
    manifest = []
    for i, case in enumerate(cases):
        target = kickoff(case["seed"] + 101 * case["restart"], case["serving"], case["sign"])
        assert [[r.key.team, r.key.id] for r in target.robots] == case["keys"]
        robots = tuple(
            Robot(r.key, *pose, *velocity)
            for r, pose, velocity in zip(
                target.robots, case["poses"], case["velocities"], strict=True
            )
        )
        start = replace(target, robots=robots, ball=Ball(*case["ball"]))
        planners = [("current", KickoffDrive), ("parallel", ParallelKickoffDrive)]
        for arm, cls in planners if i % 2 == 0 else reversed(planners):
            world = backend.reset(start)
            drive = cls(target)
            initial_velocities = [[r.vx, r.vy, r.omega] for r in world.robots]
            path = []
            for tick in range(1 + round(90 / CFG.dt)):
                path.append(snapshot(world))
                out, ready = drive.commands(world, CFG.dt)
                if ready:
                    break
                world = backend.step(out)
            name = f"{case['seed']}_{case['restart']}_{arm}"
            traces[name] = np.array(path)
            manifest.append(
                dict(
                    name=name,
                    arm=arm,
                    seed=case["seed"],
                    restart=case["restart"],
                    ready=ready,
                    seconds=round(tick * CFG.dt, 2),
                    initial_velocities=initial_velocities,
                )
            )
    np.savez_compressed(folder / "kickoff-traces.npz", **traces)
    (folder / "kickoff-replays.json").write_text(json.dumps(manifest, indent=2) + "\n")
