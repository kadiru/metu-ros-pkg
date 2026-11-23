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
from roboturk.team_kickoff import IndependentKickoffs
from roboturk.ssl_vision import detection


def run(sim, folder, cases, *, teams=False, film=False):
    boards(sim)
    backend = ContinuousWebotsBackend(sim, CFG)
    traces = {}
    manifest = []
    filmed = next((c for c in cases if c["kind"] == "half-time"), None) if film else None
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
        if teams:
            planners = [("shared", ParallelKickoffDrive), ("teams", IndependentKickoffs)]
        for arm, cls in planners if i % 2 == 0 else reversed(planners):
            name = f"{case['seed']}_{case['restart']}_{arm}"
            frames = folder / "replay-frames" / name
            if case is filmed:
                frames.mkdir(parents=True)
            world = backend.reset(start)
            drive = cls(target)
            initial_velocities = [[r.vx, r.vy, r.omega] for r in world.robots]
            path = []
            for tick in range(1 + round(90 / CFG.dt)):
                path.append(snapshot(world))
                if case is filmed and tick % 2 == 0:
                    sim.exportImage(str(frames / f"{tick // 2:05d}.jpg"), 85)
                if arm == "teams":
                    frame = detection(world, tick, world.time, backend.ball.getPosition()[2])
                    out, ready = drive.commands(world, CFG.dt, frame.detection)
                else:
                    out, ready = drive.commands(world, CFG.dt)
                if ready:
                    break
                world = backend.step(out)
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
