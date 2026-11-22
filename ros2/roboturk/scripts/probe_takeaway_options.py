"""Replay each original clean steal with each maneuver, against its original opponent."""

import argparse
import math
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import train_touch_planner as native
from train_takeaway import ROOT, WEBOTS, OLD, WORLD, MODES, historical, read, save, sha


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder", type=Path)
    p.add_argument("--workers", type=int, choices=(1, 2, 3, 4), default=2)
    a = p.parse_args()
    base = read(OLD / "policy.json")
    models = [base | dict(takeaway_tree=dict(action=i)) for i in range(len(MODES))] + [base]
    cases = historical()
    schedule = []
    starts = []
    durations = []
    for f, g, r in cases:
        for j in range(len(MODES)):
            pair = (j, len(MODES)) if r["team"] == "blue" else (len(MODES), j)
            schedule.append([g["seed"], *pair])
            starts.append(r["sample"])
            durations.append(min(g["seconds"], math.ceil(r["sample"] / 100) + 18))
    d = a.folder.resolve()
    d.mkdir(parents=True, exist_ok=False)
    cfg = dict(
        root=str(ROOT),
        models=models,
        schedule=schedule,
        seconds=90,
        cases=len(schedule),
        starts=starts,
        durations=durations,
        world=str(WORLD.relative_to(ROOT)),
        webots_sha256=sha(WEBOTS),
        press=True,
        record=False,
    )
    save(d / "launch.json", cfg)
    native.WORLD = WORLD
    with ThreadPoolExecutor(max_workers=a.workers) as pool:
        list(pool.map(lambda i: native.isolated(d, i, WEBOTS), range(len(schedule))))
    report = []
    for i in range(len(schedule)):
        f, g, r = cases[i // len(MODES)]
        prefix = (r["sample"] // 4 + 1) * 4
        original = np.load(f.with_name(f.name.replace("game", "trace").replace("json", "npz")))[
            "states"
        ]
        states = np.load(d / f"trace-{i:03}.npz")["states"]
        assert np.array_equal(original[:prefix], states[:prefix])
        game = read(d / f"game-{i:03}.json")
        (decision,) = game["takeaway_actions"]
        row = next(
            x for x in game["recoveries"] if x["sample"] == r["sample"] and x["kind"] == "takeaway"
        )
        report.append(
            dict(
                seed=g["seed"],
                action=decision["action"],
                x=decision["x"],
                matched_samples=prefix,
                **row,
            )
        )
    save(d / "options.json", report)


if __name__ == "__main__":
    main()
