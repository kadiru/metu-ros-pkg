"""Run native counterfactual training, self-play development and paired qualification."""

import argparse
import math
from pathlib import Path
from takeaway_native import launch_cases
from train_takeaway import ROOT, WEBOTS, WORLD as OLD_WORLD, read, save, sha, summarize
from roboturk.open_play_audit import recoveries
import numpy as np

DEST = ROOT / "policies/counterpass_v1"
OLD = ROOT / "policies/takeaway_v1"
WORLD = OLD_WORLD.with_name("ssl_counterpass.wbt")
CTRL = WORLD.parent.parent / "controllers/roboturk_counterpass/roboturk_counterpass.py"


def training_rows(plan):
    rows = []
    for f in sorted((ROOT / plan["source"]).glob("game-*.json")):
        g = read(f)
        if (g["blue_model"], g["yellow_model"]) != (1, 1):
            continue
        clean = [r for r in g["recoveries"] if r["kind"] == "takeaway" and r["sample"] < 2400]
        rows.extend((f, g, r) for r in clean[:2])
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=["control", "train", "dev", "evaluate"])
    p.add_argument("folder", type=Path)
    p.add_argument("--workers", type=int, choices=range(1, 9), default=4)
    p.add_argument("--resume", action="store_true")
    p.add_argument("--checkpoint", type=Path, default=DEST / "policy.json")
    a = p.parse_args()
    plan = read(DEST / "protocol.json")
    for lock in [OLD / "freeze.json", OLD / "calibration-freeze.json"] + (
        [DEST / "freeze.json"] if a.mode == "evaluate" else []
    ):
        assert all(sha(ROOT / name) == value for name, value in read(lock).items()), lock
    if a.mode == "evaluate" and sha(a.checkpoint) != sha(DEST / "policy.json"):
        raise ValueError("Evaluation requires the frozen policy")
    base = read(ROOT / plan["baseline"])
    models = [base]
    schedule = []
    starts = []
    durations = []
    refs = []

    def add(seed, b, y, target=None, seconds=90, ref=None):
        schedule.append([seed, b, y])
        starts.append(target)
        durations.append(seconds)
        refs.append(ref)

    if a.mode == "train":
        models.extend(base | {"distribution": {"action": i}} for i in (1, 2, 3))
        for f, g, r in training_rows(plan):
            for j in (1, 2, 3):
                pair = (j, 0) if r["team"] == "blue" else (0, j)
                add(g["seed"], *pair, r["sample"], math.ceil((r["sample"] + 1) / 100) + 18, f.name)
    elif a.mode == "dev":
        models.extend(read(a.checkpoint))
        for i in range(6):
            for j in range(1, len(models)):
                for b, y in ((j, j), (j, 0), (0, j)):
                    add(plan["dev_first"] + i, b, y, seconds=60)
    else:
        if a.mode == "evaluate":
            models.append(read(a.checkpoint))
        for i in range(plan["fresh_starts"]):
            if a.mode == "control":
                add(plan["fresh_first"] + i, 0, 0)
            else:
                add(plan["fresh_first"] + i, 1, 1)
                if i < plan["mixed_starts"]:
                    add(plan["fresh_first"] + i, 0, 1)
                    add(plan["fresh_first"] + i, 1, 0)
    d = a.folder.resolve()
    world = OLD_WORLD if a.mode == "control" else WORLD
    cfg = dict(
        root=str(ROOT),
        world=str(world.relative_to(ROOT)),
        models=models,
        schedule=schedule,
        cases=len(schedule),
        seconds=90,
        durations=durations,
        decision_starts=starts,
        references=refs,
        press=True,
        finish_search=True,
        record=False,
        webots_sha256=sha(WEBOTS),
    )
    launch_cases(d, cfg, WEBOTS, a.workers, a.resume)
    games = []
    for i, (seed, b, y) in enumerate(schedule):
        g = read(d / f"game-{i:03}.json")
        s = np.load(d / f"trace-{i:03}.npz")["states"]
        assert (g["seed"], g["blue_model"], g["yellow_model"]) == (seed, b, y)
        assert len(s) == 100 * durations[i] and recoveries(s, g) == g["recoveries"]
        games.append(g)
    save(d / "summary.json", summarize(games))


if __name__ == "__main__":
    main()
