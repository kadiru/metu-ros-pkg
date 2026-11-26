"""Breed runnable team policies from goal and interception returns in a native self-play league."""

import argparse
from pathlib import Path
import numpy as np
from run_recovery_attack import read, save


def population(folder, seed):
    models = read(folder / "launch.json")["models"]
    rank = sorted(
        read(folder / "returns.json"),
        key=lambda r: (r["lead_goals"] - 0.7 * r["interceptions"], r["lead_goals"]),
        reverse=True,
    )
    first, second = [models[r["candidate"]] for r in rank[:2]]
    rng = np.random.default_rng(seed)
    next_generation = [first, second]
    bounds = dict(
        run_speed=(0.4, 0.82),
        release_gap=(0.11, 0.26),
        reach=(0.69, 1.1),
        horizon=(0.69, 1.02),
        min_lead=(0.22, 0.36),
        sync_angle=(0.16, 0.42),
        cut=(0.0, 0.72),
        progress=(-0.19, 0.38),
        forecast=(0.30, 0.92),
        length_weight=(0.25, 0.86),
        receive_escape=(0.30, 1.05),
    )
    for _ in range(3):
        child = first.copy()
        for k, (lo, hi) in bounds.items():
            blend = rng.uniform(0.50, 0.95)
            value = blend * first[k] + (1 - blend) * second[k] + rng.normal(0, 0.12 * (hi - lo))
            child[k] = round(float(np.clip(value, lo, hi)), 5)
        next_generation.append(child)
    return next_generation


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("league", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--seed", type=int, default=82813)
    a = p.parse_args()
    save(a.output, population(a.league, a.seed))
