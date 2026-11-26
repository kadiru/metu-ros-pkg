"""Mutate first-touch policies using native self-play returns and the five known recoveries."""

import argparse
from pathlib import Path
import numpy as np
from train_recovery_touch import read, save


def evolve(league, known, seed, generation):
    models = read(league / "launch.json")["models"]
    returns = read(league / "returns.json")
    repaired = [0] * len(models)
    if known:
        for f in sorted(known.glob("game-*.json")):
            g = read(f)
            row = g["opening"]
            original = "blue" if g["blue_model"] != len(models) else "yellow"
            if row.get("chain", {}).get("team") == original and row["qualified"]:
                repaired[g[original + "_model"]] += 1

    def rank(j):
        r = returns[j]
        return (r["goals"] - 1.4 * r["prepass_losses"] + 3) / (r["games"] + 5) + 0.035 * repaired[j]

    order = sorted(range(len(models)), key=rank, reverse=True)
    parents = [models[j] | dict(offer=True) for j in order[:2]]
    rng = np.random.default_rng(seed)
    kids = list(parents)
    spans = {
        "release_value": (0.75, 1.85),
        "look": (0.025, 0.13),
        "rush_weight": (0.65, 2.45),
        "shield_bias": (0.05, 0.65),
        "side_bias": (0.18, 0.93),
        "side_cost": (-0.12, 0.36),
        "follow": (0.32, 0.86),
        "grip": (14.0, 27.0),
        "brake": (0.45, 0.96),
        "cruise": (0.38, 0.83),
        "pivot": (2.6, 6.2),
    }
    for j in range(3):
        child = dict(parents[j % 2])
        for key, (lo, hi) in spans.items():
            child[key] = round(
                float(
                    np.clip(
                        child[key] + rng.normal(0, (hi - lo) * (0.20 if generation == 1 else 0.12)),
                        lo,
                        hi,
                    )
                ),
                5,
            )
        kids.append(child)
    return kids


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("league", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--known", type=Path)
    p.add_argument("--seed", type=int, default=8141)
    p.add_argument("--generation", type=int, default=1)
    a = p.parse_args()
    save(a.output, evolve(a.league, a.known, a.seed, a.generation))
