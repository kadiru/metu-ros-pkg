"""Native ten-robot ball-placement scenarios and episodic controller fitting."""

from dataclasses import replace
import json
import math
from pathlib import Path
import random

import numpy as np
from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.five_roles import CFG, TEAMS, kickoff
from roboturk.model import Ball
from roboturk.placement import PlacementPlay, xy


def scenario(root, seed, crossing=None):
    rng = random.Random(seed)
    team = TEAMS[seed % 2]
    sign = rng.choice((-1, 1))
    world = kickoff(seed, team, sign)
    if crossing is not None:
        panel = json.loads((root / "media/autoref/passive.json").read_text())
        event = next(
            e["protobuf"]["ballLeftFieldTouchLine"]
            for m in panel
            if m["seed"] == seed
            for e in m["boundaries"]
            if e["tick"] == crossing
        )
        team = "blue" if event["byTeam"] == "YELLOW" else "yellow"
        with np.load(root / f"media/five-roles/matches/{seed}/positions.npz") as f:
            state = f["states"][crossing]
        robots = tuple(
            replace(r, x=x, y=y, yaw=a)
            for r, (x, y, a) in zip(world.robots, state[4:].reshape(-1, 3), strict=True)
        )
        target = (event["location"]["x"], math.copysign(2.8, event["location"]["y"]))
        return (
            replace(
                world, robots=robots, ball=Ball(*state[:4]), attack_sign=1 if seed % 2 == 0 else -1
            ),
            team,
            target,
        )
    side = rng.choice((-1, 1))
    x = rng.uniform(-2.85, 2.85)
    y = side * rng.uniform(3.04, 3.19)
    target = (x + rng.uniform(-0.55, 0.55), side * rng.uniform(2.35, 2.8))
    ball = Ball(x, y, rng.uniform(-0.06, 0.06), side * rng.uniform(-0.04, 0.06))
    placer, marker = rng.randint(1, 4), rng.randint(1, 4)
    # Opponent begins inside the exclusion corridor and has to clear it.
    points = {
        (team, placer): (
            x + rng.choice((-1, 1)) * rng.uniform(0.5, 1.0),
            y - side * rng.uniform(0.35, 0.85),
        ),
        (TEAMS[1 - seed % 2], marker): (
            target[0] + rng.uniform(-0.23, 0.23),
            target[1] - side * 0.10,
        ),
    }
    robots = []
    occupied = list(points.values())
    for r in world.robots:
        p = points.get((r.key.team, r.key.id))
        if p is None:
            p = xy(r)
            while min(math.dist(p, q) for q in occupied) < 0.27:
                p = (rng.uniform(-3.6, 3.6), rng.uniform(-2.1, 2.1))
            occupied.append(p)
        robots.append(replace(r, x=p[0], y=p[1], yaw=rng.uniform(-math.pi, math.pi)))
    return replace(world, ball=ball, robots=tuple(robots)), team, target


def boards(sim):
    for side in (-1, 1):
        sim.getRoot().getField("children").importMFNodeFromString(
            -1,
            f'Solid {{ name "sideline board {side}" translation 0 {side * 3.48} 0.06 '
            "children [ Shape { appearance PBRAppearance { baseColor .24 .25 .26 } "
            "geometry Box { size 10 .035 .12 } } ] boundingObject Box { size 10 .035 .12 } }",
        )
        sim.getRoot().getField("children").importMFNodeFromString(
            -1,
            f'Solid {{ name "end board {side}" translation {side * 4.98} 0 0.06 '
            "children [ Shape { appearance PBRAppearance { baseColor .24 .25 .26 } "
            "geometry Box { size .035 7 .12 } } ] boundingObject Box { size .035 7 .12 } }",
        )


def rollout(backend, root, seed, weights, crossing=None):
    world, team, target = scenario(root, seed, crossing)
    w = backend.reset(world)
    play = PlacementPlay(w, team, target, weights)
    stable = 0
    for tick in range(750):
        out, _ = play.place(w)
        w = backend.step(out)
        error = math.dist(xy(w.ball), target)
        clearance = min(math.dist(xy(r), xy(w.ball)) for r in w.robots)
        ok = (
            play.phase == "retreat"
            and error < 0.12
            and clearance > 0.59
            and math.hypot(w.ball.vx, w.ball.vy) < 0.035
        )
        stable = stable + 1 if ok else 0
        if stable >= 8 or abs(w.ball.y) > 3.6:
            break
    return dict(
        seed=seed,
        crossing=crossing,
        team=team,
        placer=play.placer.id,
        success=stable >= 8,
        seconds=round(w.time, 2),
        error=round(error, 5),
        phase=play.phase,
        speed=math.hypot(w.ball.vx, w.ball.vy),
        ball=xy(w.ball),
        target=target,
        clearance=round(clearance, 4),
    )


def run(sim, folder, cfg, receiver, vision):
    spec = cfg["placement_batch"]
    root = Path(cfg["root"])
    backend = ContinuousWebotsBackend(sim, CFG)
    boards(sim)
    policy = root / "policies/placement_v1"
    protocol = json.loads((policy / "protocol.json").read_text())
    if spec["mode"] in ("official", "demo"):
        from placement_official import official

        result = official(sim, backend, root, folder, spec, receiver, vision)
    elif spec["mode"] == "train":
        rng = np.random.default_rng(1719)
        mean = np.array([1.25, 0.65, 2.15, 0.035, 0.019])
        sigma = np.array([0.38, 0.22, 0.77, 0.014, 0.010])
        low = [0.6, 0.25, 0.85, 0.012, 0.007]
        high = [1.9, 1.1, 4.3, 0.065, 0.035]
        history = []
        for gen in range(spec.get("generations", 3)):
            seeds = [
                protocol["training_seed_base"] + gen * 20 + i for i in range(spec.get("starts", 3))
            ]
            candidates = [
                mean,
                *np.clip(rng.normal(mean, sigma, (spec.get("candidates", 6) - 1, 5)), low, high),
            ]
            rows = []
            for p in candidates:
                episodes = [rollout(backend, root, s, p.tolist()) for s in seeds]
                score = np.mean(
                    [10 * e["success"] - 0.12 * e["seconds"] - 4 * e["error"] for e in episodes]
                )
                rows.append(dict(weights=p.tolist(), score=float(score), episodes=episodes))
            elite = sorted(rows, key=lambda r: -r["score"])[:2]
            mean = np.mean([r["weights"] for r in elite], axis=0)
            sigma = np.maximum(sigma * 0.60, np.std([r["weights"] for r in elite], axis=0))
            history.append(rows)
            (folder / "history.json").write_text(json.dumps(history))
        result = dict(weights=elite[0]["weights"], generations=history)
    else:
        weights = spec.get("weights") or json.loads((policy / "actor.json").read_text())["weights"]
        trials = [(s, None) for s in spec.get("seeds", [])] + spec.get("archived", [])
        result = [rollout(backend, root, s, weights, c) for s, c in trials]
    (folder / "placement.json").write_text(json.dumps(result, indent=2) + "\n")
