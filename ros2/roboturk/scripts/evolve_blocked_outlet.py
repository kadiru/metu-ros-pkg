"""Breed the next shared team population using only native self-play returns."""

import argparse
import json
from pathlib import Path
import numpy as np


def breed(folder, seed, count):
    table = json.loads((folder / "returns.json").read_text())
    models = json.loads((folder / "launch.json").read_text())["models"]
    top = sorted(table, key=lambda r: (r["goals"], r["receives"]), reverse=True)[:2]
    parents = [models[r["candidate"]] for r in top]
    bounds = dict(
        bias=(0.48, 1.08),
        block=(1.8, 3.6),
        lane=(0.48, 1.4),
        cover=(0.78, 1.25),
        tracking=(0.42, 0.76),
        escape=(0.45, 1.25),
        turn=(3.2, 7.2),
        support_x=(-0.12, 1.6),
        support_y=(0.96, 1.8),
        finish=(2.5, 9.0),
        slide=(0.12, 0.92),
        advance=(0.75, 1.55),
        shot_clear=(0.10, 0.20),
        lookahead=(0.05, 0.23),
        open_penalty=(4.0, 13.0),
    )
    rng = np.random.default_rng(seed)
    children = list(parents)
    for _ in range(count - len(parents)):
        child = parents[0].copy()
        for key, (lo, hi) in bounds.items():
            if key not in child:
                continue
            blend = rng.uniform(0, 1)
            value = blend * parents[0][key] + (1 - blend) * parents[1][key]
            child[key] = round(float(np.clip(value + rng.normal(0, 0.14 * (hi - lo)), lo, hi)), 5)
        weights = (np.asarray(parents[0]["receiver"]) + np.asarray(parents[1]["receiver"])) / 2
        child["receiver"] = (
            (weights + rng.normal(0, [0.32, 0.12, 0.18, 0.10, 0.10])).round(5).tolist()
        )
        children.append(child)
    return children


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("league", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--seed", type=int, default=817703)
    p.add_argument("--count", type=int, default=6)
    a = p.parse_args()
    if a.count < 2:
        p.error("At least two parents are required")
    a.output.write_text(json.dumps(breed(a.league, a.seed, a.count), indent=2) + "\n")
