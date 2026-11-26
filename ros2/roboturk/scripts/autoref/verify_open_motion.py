"""Recheck released match counts against native states and the original referee payloads."""

import argparse
import json
from itertools import groupby
from pathlib import Path
import numpy as np
from train_open_motion import PROTOCOL, read_game
from open_motion_metrics import stats
from judge_open_motion import CRASHES
from verify import load_events


def verify(folder):
    comparison = json.loads((folder / "comparison.json").read_text())
    lo, hi = PROTOCOL["fresh_seeds_inclusive"]
    assert [r["seed"] for r in comparison["pairs"]] == list(range(lo, hi + 1))
    differences = {metric: [] for metric in ["crashes", "passes", "goals"]}
    totals = {
        team: dict(crashes=0, passes=0, goals=0, attempts=0) for team in ("baseline", "candidate")
    }
    for pair in comparison["pairs"]:
        openings = []
        for side in totals:
            run = folder / str(pair["seed"]) / side
            log, states = read_game(run)
            assert (
                log["seed"] == pair["seed"]
                and log["seconds"] == PROTOCOL["seconds_per_fresh_start"]
            )
            assert len(states) == round(log["seconds"] / log["dt"]) == len(log["frames"])
            openings.append([*states[0], *np.ravel(log["frames"][0]["opening_velocities"])])
            raw = json.loads((run / "official/result.json").read_text())
            stats_now = stats(log, states)
            assert all(raw[k] == v for k, v in stats_now.items())
            assert sum(log["score"]) == len(raw["goals"])
            calls = []
            for n, group in groupby(enumerate(log["frames"]), key=lambda x: x[1]["restart"]):
                indices = [i for i, _ in group]
                for t, k, p in load_events(run / "official" / f"segment-{n}" / "tigers.log"):
                    tick = round((t - 1800000000.0) / log["dt"])
                    assert indices[0] <= tick <= indices[-1]
                    calls.append(dict(tick=tick, segment=n, kind=k, event=p))
            assert (
                calls == raw["detector_events"]
                and [e for e in calls if e["kind"] in CRASHES] == raw["crashes"]
            )
            counts = dict(
                crashes=len(raw["crashes"]),
                goals=len(raw["goals"]),
                passes=len(raw["passes"]),
                attempts=raw["attempts"],
            )
            assert counts == pair[side]
            for metric in counts:
                totals[side][metric] += counts[metric]
        assert np.array_equal(*openings)
        for metric in differences:
            differences[metric].append(pair["candidate"][metric] - pair["baseline"][metric])
    assert totals == comparison["totals"]
    return dict(
        totals=totals,
        paired_games={
            k: dict(fewer=sum(x < 0 for x in a), same=a.count(0), more=sum(x > 0 for x in a))
            for k, a in differences.items()
        },
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("fresh", type=Path)
    print(json.dumps(verify(p.parse_args().fresh), indent=2))
