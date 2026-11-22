"""Replay each original clean steal with each maneuver, against its original opponent."""

import argparse
import math
from pathlib import Path
from takeaway_native import launch_cases
import numpy as np
from roboturk.takeaway_outlet import EXTRA
from train_takeaway import ROOT, WEBOTS, OLD, WORLD, MODES, historical, read, save, sha


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder", type=Path)
    p.add_argument("--workers", type=int, choices=(1, 2, 3, 4), default=2)
    p.add_argument("--extra", action="store_true")
    p.add_argument("--finish-search", action="store_true")
    p.add_argument("--sprint-search", action="store_true")
    p.add_argument("--release-search", action="store_true")
    p.add_argument("--resume", action="store_true")
    a = p.parse_args()
    base = read(OLD / "policy.json")
    actions = (
        list(range(len(MODES), len(MODES) + len(EXTRA))) if a.extra else list(range(len(MODES)))
    )
    models = [base | dict(takeaway_tree=dict(action=i)) for i in actions] + [base]
    profiles = []
    searches = [name for name in ("finish", "sprint", "release") if getattr(a, name + "_search")]
    if len(searches) > 1:
        p.error("Select one training search")
    if searches:
        name = searches[0] + "-search"
        profiles = read(ROOT / f"policies/takeaway_v1/{name}.json")["profiles"]
        actions = [9] * len(profiles)
        models = [base | dict(takeaway_tree=dict(action=9), escape_finish=v) for v in profiles] + [
            base
        ]
    cases = historical()
    schedule = []
    starts = []
    durations = []
    for f, g, r in cases:
        for j in range(len(actions)):
            pair = (j, len(actions)) if r["team"] == "blue" else (len(actions), j)
            schedule.append([g["seed"], *pair])
            starts.append(r["sample"])
            durations.append(min(g["seconds"], math.ceil(r["sample"] / 100) + 18))
    d = a.folder.resolve()
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
        finish_search=bool(profiles),
    )
    launch_cases(d, cfg, WEBOTS, a.workers, a.resume)
    report = []
    for i in range(len(schedule)):
        f, g, r = cases[i // len(actions)]
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
                **(dict(profile=i % len(profiles)) if profiles else {}),
                **row,
            )
        )
    save(d / "options.json", report)


if __name__ == "__main__":
    main()
