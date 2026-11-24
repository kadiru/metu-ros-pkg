"""Paired live-ball recovery trials with the normal ten-player open-play controller."""

from dataclasses import replace
import hashlib
from itertools import combinations
import json
import math
from pathlib import Path
import random

import numpy as np
from roboturk import field
from roboturk.errt import ERRTFallback
from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.five_play import open_play
from roboturk.five_roles import CFG, TEAMS, FiveRoles, kickoff
from roboturk.model import Ball, possession
from roboturk.open_motion import Steering, observed_velocity


def opening(seed):
    rng = random.Random(seed)
    side = 1 if seed % 2 == 0 else -1
    favored = TEAMS[(seed // 2) % 2]
    world = kickoff(seed, favored, 1 if seed % 4 in (0, 3) else -1)
    x = rng.choice((-1, 1)) * (rng.uniform(3.7, 4.12) if seed % 5 == 0 else rng.uniform(0.2, 3.45))
    ball = Ball(
        x,
        side * rng.uniform(2.97, 3.012),
        rng.uniform(-0.22, 0.22),
        side * rng.uniform(-0.025, 0.045),
    )
    robots = []
    for r in world.robots:
        if r.key.id == 1:
            near = r.key.team == favored
            dx = rng.uniform(0.28, 0.57) * (1 if (seed // 4) % 2 else -1)
            rx = x + (dx if near else -2.1 * dx)
            rx = max(-4.24, min(4.24, rx))
            ry = side * (
                abs(ball.y) - (rng.uniform(0.47, 0.74) if near else rng.uniform(1.40, 1.65))
            )
            # Other kickoff players occupy the central lanes. Shift this player along
            # the line until the opening is free of overlapping bodies.
            while any(math.hypot(q.x - rx, q.y - ry) < 0.32 for q in world.robots if q.key.id != 1):
                ry += side * 0.15
            r = replace(r, x=rx, y=ry, yaw=math.atan2(ball.y - ry, x - rx))
        robots.append(r)
    return replace(world, ball=ball, robots=tuple(robots))


def frame_gap(r):
    x, y = abs(r.x), abs(r.y)
    start = field.LENGTH / 2 + field.LINE_WIDTH / 2
    rear = start + field.GOAL_DEPTH
    half = field.GOAL_WIDTH / 2

    def rect(left, right, bottom, top):
        return math.hypot(max(0, left - x, x - right), max(0, bottom - y, y - top))

    return (
        min(
            rect(start, field.LENGTH / 2 + field.WALL_MARGIN, half, half + field.GOAL_WALL),
            rect(rear, rear + field.GOAL_WALL, 0, half),
        )
        - CFG.robot_radius
    )


def trial(sim, backend, cfg, seed, weights, policy, frames):
    initial = opening(seed)
    w = backend.reset(initial)
    start = [[r.key.team, r.key.id, r.x, r.y, r.yaw, r.vx, r.vy, r.omega] for r in w.robots]
    start.append([w.ball.x, w.ball.y, w.ball.vx, w.ball.vy, w.attack_sign])
    digest = hashlib.sha256(json.dumps(start).encode()).hexdigest()
    if frames:
        camera = sim.getFromDef("CAMERA")
        camera.getField("position").setSFVec3f([w.ball.x, math.copysign(2.55, w.ball.y), 4.5])
    tactics = FiveRoles()
    fallback = (
        ERRTFallback(seed, cfg.get("commit_waypoints", False)) if cfg.get("errt_fallback") else None
    )
    previous = {}
    last_holder = None
    dwell = 0.0
    recovered = None
    crossed = outside_hold = False
    low = dict(pair=10.0, board=10.0, goal=10.0)
    counts = dict.fromkeys(low, 0)
    active = set()
    counts_planner = {}
    end = "timeout"
    for tick in range(round(cfg["seconds"] / CFG.dt)):
        if frames and tick % 2 == 0:
            sim.exportImage(str(frames / f"{tick // 2:04d}.jpg"), 90)
        measured = observed_velocity(w.robots, previous, CFG.dt)
        previous = {r.key: (r.x, r.y) for r in w.robots}
        cmd, passes, _, _ = open_play(
            w,
            tactics,
            weights,
            measured,
            policy,
            cfg["touchline_recovery"],
            cfg.get("trajectory_planner", False),
            counts_planner,
            fallback=fallback,
        )
        w = backend.step(cmd)
        for k, _, _ in backend.last_kick_events:
            tactics.flight = passes.get(k)
        for sample in backend.last_substep_worlds:
            contacts = set()

            def check(kind, key, gap):
                low[kind] = min(low[kind], gap)
                if gap <= 0.001:
                    contacts.add((kind, key))

            for r in sample.robots:
                check(
                    "board",
                    r.key,
                    min(
                        field.LENGTH / 2 + field.WALL_MARGIN - abs(r.x),
                        field.WIDTH / 2 + field.WALL_MARGIN - abs(r.y),
                    )
                    - CFG.robot_radius,
                )
                check("goal", r.key, frame_gap(r))
            for a, b in combinations(sample.robots, 2):
                check(
                    "pair", (a.key, b.key), math.hypot(a.x - b.x, a.y - b.y) - 2 * CFG.robot_radius
                )
            for kind, _ in contacts - active:
                counts[kind] += 1
            active = contacts
            if abs(sample.ball.y) > CFG.ball_exit_y or abs(sample.ball.x) > CFG.ball_exit_x:
                end = "out"
                break
            crossed |= any(abs(r.y) > field.WIDTH / 2 for r in sample.robots)
            holders = [r for r in sample.robots if possession(sample, r.key, CFG)]
            holder = holders[0] if len(holders) == 1 else None
            key = holder.key if holder else None
            dwell = (dwell if key == last_holder else 0) + backend.basic_ms / 1000 if key else 0
            last_holder = key
            if holder:
                outside_hold |= abs(holder.y) > field.WIDTH / 2
                if not recovered and dwell >= 0.20 and abs(sample.ball.y) <= field.WIDTH / 2 - 0.25:
                    recovered = dict(team=key.team, id=key.id, seconds=round(sample.time, 3))
        if end == "out":
            break
    return dict(
        seed=seed,
        start=digest,
        initial=start,
        recovered=recovered,
        crossed=crossed,
        outside_hold=outside_hold,
        contacts=counts,
        planner=counts_planner,
        min_gap_m=low,
        end=end,
    )


def run(sim, folder, cfg):
    root = Path(cfg["root"])
    raw = json.loads((root / "policies/recipient_3v3_v1/selectors.json").read_text())
    weights = {t: np.asarray(raw[str(901 + i)]) for i, t in enumerate(TEAMS)}
    policy = Steering(**cfg["open_play_policy"])
    backend = ContinuousWebotsBackend(sim, CFG)
    frames = folder / "frames" if cfg.get("frame_images") else None
    if frames:
        if cfg["touchline_trials"] != 1:
            raise ValueError("record a single trial per frame directory")
        frames.mkdir()
    trials = []
    for seed in range(cfg["seed"], cfg["seed"] + cfg["touchline_trials"]):
        trials.append(trial(sim, backend, cfg, seed, weights, policy, frames))
        (folder / "touchline.json").write_text(json.dumps(trials, indent=2) + "\n")
