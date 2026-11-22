"""Compare passing-lane and shot-lane interception policies in full native self-play."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
import train_touch_planner as native
from train_takeaway import ROOT, OLD, DEST, WORLD, WEBOTS, read, save, sha
from roboturk.open_play_audit import recoveries


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder", type=Path)
    p.add_argument("--workers", type=int, choices=(1, 2, 3, 4), default=2)
    a = p.parse_args()
    d = a.folder.resolve()
    d.mkdir(parents=True, exist_ok=False)
    base = read(OLD / "policy.json")
    protocol = read(DEST / "interception-search.json")
    models = [base] + [base | dict(takeaway_press=profile) for profile in protocol["profiles"]]
    schedule = []
    for i in range(protocol["training_starts"]):
        seed = protocol["first"] + i
        for j in range(len(models)):
            schedule.append([seed, j, j])
            if i % 2 == 0:
                schedule.append([seed, j, (j + 1 + (i // 2) % 3) % len(models)])
    cfg = dict(
        root=str(ROOT),
        schedule=schedule,
        models=models,
        cases=len(schedule),
        seconds=protocol["seconds"],
        world=str(WORLD.relative_to(ROOT)),
        webots_sha256=sha(WEBOTS),
        press=True,
        record=False,
    )
    save(d / "launch.json", cfg)
    native.WORLD = WORLD
    with ThreadPoolExecutor(max_workers=a.workers) as pool:
        list(pool.map(lambda i: native.isolated(d, i, WEBOTS), range(len(schedule))))
    stats = [dict(appearances=0, clean=0, goals=0, all_recovery_goals=0, score=0) for _ in models]
    first = {}
    for i, (seed, blue, yellow) in enumerate(schedule):
        g = read(d / f"game-{i:03}.json")
        s = np.load(d / f"trace-{i:03}.npz")["states"]
        assert g["initial_world"] == first.setdefault(seed, g["initial_world"])
        assert len(s) == 100 * cfg["seconds"] and recoveries(s, g) == g["recoveries"]
        for team, j in [("blue", blue), ("yellow", yellow)]:
            own = [r for r in g["recoveries"] if r["team"] == team]
            clean = [r for r in own if r["kind"] == "takeaway"]
            a = stats[j]
            a["appearances"] += 1
            a["clean"] += len(clean)
            a["goals"] += sum("goal" in r for r in clean)
            a["all_recovery_goals"] += sum("goal" in r for r in own)
            a["score"] += g["score"].get(team, 0)

    def value(j):
        a = stats[j]
        return a["goals"] / a["appearances"] + 0.25 * a["goals"] / max(1, a["clean"])

    best = max(range(len(models)), key=value)
    save(
        d / "summary.json",
        dict(selected=best, stats=stats, values=[value(j) for j in range(len(models))]),
    )
    save(d / "selected.json", models[best])


if __name__ == "__main__":
    main()
