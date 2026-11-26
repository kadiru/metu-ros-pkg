"""Paired crowded touchline starts and native reward-based steering search."""

from dataclasses import replace
import json
import math
import random

import numpy as np
from roboturk.five_roles import TEAMS, kickoff
from roboturk.model import Ball
from roboturk.placement import PlacementPlay, xy


def crowded(seed):
    rng = random.Random(seed)
    team = TEAMS[seed % 2]
    other = TEAMS[1 - seed % 2]
    side = rng.choice((-1, 1))
    mirror = rng.choice((-1, 1))
    x = rng.uniform(-2.5, 2.5)
    y = rng.uniform(3.08, 3.27)
    dx = mirror * rng.uniform(0.12, 0.78)
    dy = rng.uniform(1.38, 2.05)
    t = rng.uniform(0.38, 0.62)
    award = (x + mirror * rng.uniform(-0.18, 0.14), side * 2.8)
    w = kickoff(seed, team, rng.choice((-1, 1)))
    jersey, blocker = rng.randint(1, 4), rng.randint(1, 4)
    locations = {
        (team, jersey): (x + dx, side * (y - dy)),
        (other, blocker): (x + t * dx + rng.uniform(-0.09, 0.09), side * (y - t * dy)),
    }
    used = list(locations.values())
    robots = []
    for r in w.robots:
        spot = locations.get((r.key.team, r.key.id))
        if spot is None:
            spot = xy(r)
            while (
                min(math.dist(spot, q) for q in used) < 0.32
                or math.dist(spot, (x, side * y)) < dy + 0.22
            ):
                spot = (rng.uniform(-3.95, 3.95), rng.uniform(-1.85, 1.85))
            used.append(spot)
        robots.append(replace(r, x=spot[0], y=spot[1], yaw=rng.uniform(-math.pi, math.pi)))
    return replace(w, robots=tuple(robots), ball=Ball(x, side * y)), team, award


def nearest_pair(w):
    return min(
        math.hypot(a.x - b.x, a.y - b.y) for i, a in enumerate(w.robots) for b in w.robots[:i]
    )


def trial(backend, seed, weights, traffic):
    start, team, target = crowded(seed)
    w = backend.reset(start)
    play = PlacementPlay(w, team, target, weights, traffic)
    minimum = nearest_pair(w)
    settled = 0
    for _ in range(750):
        out, _ = play.place(w)
        w = backend.step(out)
        minimum = min(minimum, *(nearest_pair(sample) for sample in backend.last_substep_worlds))
        error = math.dist(xy(w.ball), target)
        ok = (
            play.phase == "retreat"
            and error < 0.12
            and math.hypot(w.ball.vx, w.ball.vy) < 0.035
            and min(math.dist(xy(r), xy(w.ball)) for r in w.robots) > 0.59
        )
        settled = settled + 1 if ok else 0
        if settled >= 8:
            break
    reward = (
        12 * (settled >= 8) - 15 * (minimum < 0.183) - 30 * max(0, 0.235 - minimum) - 0.1 * w.time
    )
    return dict(
        seed=seed,
        success=settled >= 8,
        seconds=round(w.time, 2),
        error=error,
        min_center_m=minimum,
        reward=reward,
    )


def fit(backend, root, folder, spec):
    weights = json.loads((root / "policies/placement_v1/actor.json").read_text())["weights"]
    protocol = json.loads((root / "policies/placement_v2/protocol.json").read_text())
    train = protocol["training_starts"][0]
    rng = np.random.default_rng(5659)
    mean = np.array([0.48, 0.30, 0.56])
    sd = np.array([0.16, 0.054, 0.14])
    history = []
    for generation in range(3):
        seeds = range(train + generation * 8, train + generation * 8 + 4)
        population = [
            mean,
            *np.clip(rng.normal(mean, sd, (5, 3)), [0.23, 0.235, 0.30], [0.88, 0.46, 0.85]),
        ]
        records = []
        for params in population:
            runs = [trial(backend, s, weights, params.tolist()) for s in seeds]
            records.append(
                dict(
                    traffic=params.tolist(),
                    episodes=runs,
                    score=np.mean([e["reward"] for e in runs]),
                )
            )
        elite = sorted(records, key=lambda r: -r["score"])[:2]
        history.append(records)
        mean = np.mean([r["traffic"] for r in elite], axis=0)
        sd *= 0.63
        (folder / "history.json").write_text(json.dumps(history))
    selected = elite[0]["traffic"]
    lo, hi = protocol["development_starts"]
    development = {
        name: [trial(backend, s, weights, params) for s in range(lo, hi + 1)]
        for name, params in [("baseline", None), ("candidate", selected)]
    }
    return dict(traffic=selected, history=history, development=development)


def replay_misses(backend, root, spec):
    """Replay unconfirmed awards and measure who is still inside the clearance circle."""
    from roboturk.model import Robot, RobotKey, World

    weights = json.loads((root / "policies/placement_v1/actor.json").read_text())["weights"]
    traffic = json.loads((root / "policies/placement_v2/actor.json").read_text())["traffic"]
    records = []
    for row in spec["trials"]:
        robots = tuple(
            Robot(RobotKey(t, n), x, y, yaw) for t, n, x, y, yaw in row["opening_robots"]
        )
        world = World(
            0,
            robots,
            Ball(*row["initial_ball"], *row["opening_ball_velocity"]),
            attack_sign=row.get("attack_sign", 1),
        )
        w = backend.reset(world)
        play = PlacementPlay(w, row["team"], row["target"], weights, traffic)
        trace = []
        for i in range(750):
            out, roles = play.place(w)
            w = backend.step(out)
            if i % 25 == 24:
                closest = min(w.robots, key=lambda r: math.dist(xy(r), xy(w.ball)))
                trace.append(
                    dict(
                        time=w.time,
                        phase=play.phase,
                        error_m=math.dist(xy(w.ball), row["target"]),
                        ball_speed=math.hypot(w.ball.vx, w.ball.vy),
                        closest=[closest.key.team, closest.key.id],
                        role=roles[closest.key].role,
                        clearance_m=math.dist(xy(closest), xy(w.ball)),
                        robot_xy=xy(closest),
                        desired_xy=roles[closest.key].target,
                    )
                )
        records.append(dict(seed=row["seed"], trace=trace))
    return records
