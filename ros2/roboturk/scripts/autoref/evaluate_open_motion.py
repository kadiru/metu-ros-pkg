"""Run and adjudicate the registered pairs for an observed-motion steering policy."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import numpy as np
from judge_open_motion import judge, paths
from train_open_motion import ROOT, PROTOCOL, play, read_game, save


def summaries(pairs, dest):
    rows = []
    for a, b, start in pairs:
        old, baseline = read_game(a)
        new, candidate = read_game(b)
        assert old["seed"] == new["seed"] and old["seconds"] == new["seconds"]
        assert np.array_equal(baseline[: start + 1], candidate[: start + 1])
        assert old["frames"][0]["opening_velocities"] == new["frames"][0]["opening_velocities"]
        row = dict(seed=old["seed"], prefix_frames=start + 1)
        for name, game, folder in [("baseline", old, a), ("candidate", new, b)]:
            r = json.loads((folder / "official/result.json").read_text())
            assert r["frames"] == len(game["frames"]) and sum(r["score"]) == len(r["goals"])
            row[name] = dict(
                crashes=len(r["crashes"]),
                passes=len(r["passes"]),
                goals=len(r["goals"]),
                attempts=r["attempts"],
            )
        rows.append(row)
    result = dict(
        pairs=rows,
        totals={
            team: {m: sum(r[team][m] for r in rows) for m in rows[0][team]}
            for team in ("baseline", "candidate")
        },
    )
    save(dest / "comparison.json", result)
    print(result["totals"], flush=True)


def run(args):
    dest = args.output.resolve()
    dest.mkdir(parents=True, exist_ok=False)
    actor = args.actor.resolve()
    if args.phase == "incidents":
        schedule = [
            (str(n), PROTOCOL["incident_seed"], c["end_tick"] // 25, c["switch_tick"], True)
            for n, c in enumerate(PROTOCOL["incidents"])
        ]
    else:
        seeds = (
            PROTOCOL["development_seeds"]
            if args.phase == "development"
            else range(
                PROTOCOL["fresh_seeds_inclusive"][0], 1 + PROTOCOL["fresh_seeds_inclusive"][1]
            )
        )
        schedule = [
            (
                str(s),
                s,
                30 if args.phase == "development" else PROTOCOL["seconds_per_fresh_start"],
                0,
                args.phase == "fresh" and i == 0,
            )
            for i, s in enumerate(seeds)
        ]
    pairs = []
    with ThreadPoolExecutor(1) as worker:
        pending = []
        for label, seed, seconds, start, record in schedule:
            case = dest / label
            case.mkdir()
            for side, policy in [("baseline", None), ("candidate", actor)]:
                folder = case / side
                play(folder, seed, seconds, policy, start if policy else 0, record)
                pending.append(
                    worker.submit(
                        judge,
                        folder,
                        args.distribution.resolve(),
                        args.classes.resolve(),
                        args.java.resolve(),
                    )
                )
                for job in pending:
                    if job.done():
                        job.result()
            pairs.append((case / "baseline", case / "candidate", start))
        for job in pending:
            job.result()
    summaries(pairs, dest)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("phase", choices=("incidents", "development", "fresh"))
    p.add_argument("output", type=Path)
    p.add_argument("--actor", type=Path, default=ROOT / "policies/open_motion_v1/actor.json")
    paths(p)
    run(p.parse_args())
