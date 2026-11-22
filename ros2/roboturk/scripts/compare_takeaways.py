"""Audit paired fresh starts, clean recoveries, the unchanged prefix and field limits."""

import argparse
from collections import Counter
from pathlib import Path
import numpy as np
from train_takeaway import read, save, summarize
from summarize_open_play import geometry_flags


def all_games(folder):
    return [(read(f), f) for f in sorted(folder.glob("game-*.json"))]


def trace(path):
    return np.load(path.with_name(path.name.replace("game-", "trace-").replace(".json", ".npz")))[
        "states"
    ]


def compare(control, trained):
    old = all_games(control)
    new = all_games(trained)
    previous = {g["seed"]: (g, f) for g, f in old}
    pairs = []
    geo = {}
    scores = Counter()
    mixed = []
    by_kind = {}
    for g, f in old + new:
        blue, yellow = g["blue_model"], g["yellow_model"]
        arm = "mixed" if blue != yellow else "trained" if blue else "previous"
        c = geo.setdefault(arm, Counter())
        types = by_kind.setdefault(arm, Counter())
        states = trace(f)
        outside, close = geometry_flags(states)
        assert (
            int(outside.sum()) == g["stats"]["exits"] and int(close.sum()) == g["stats"]["contacts"]
        )
        c.update(
            outside_samples=int(outside.sum()),
            close_samples=int(close.sum()),
            outside_games=int(outside.any()),
            close_games=int(close.any()),
        )
        clean = [r for r in g["recoveries"] if r["kind"] == "takeaway"]
        for r in clean:
            types[r["outcome"]] += 1
            if "goal" in r:
                span = slice(r["sample"], r["goal"] + 1)
                c["flagged_clean_goals"] += int(outside[span].any() or close[span].any())
        if arm == "previous":
            continue
        ref, ref_file = previous[g["seed"]]
        assert g["initial_world"] == ref["initial_world"]
        data = trace(ref_file)
        active = [a for a in g["takeaway_actions"] if a["action"]]
        for a in g["takeaway_actions"]:
            assert any(
                r["sample"] == a["sample"]
                and r["team"] == a["team"]
                and r["winner"] % 3 == a["actor"]
                for r in clean
            )
        prefix = 4 * (active[0]["sample"] // 4 + 1) if active else len(data)
        assert np.array_equal(states[:prefix], data[:prefix]), (g["seed"], prefix)
        if arm == "mixed":
            t = "blue" if blue else "yellow"
            z = "yellow" if blue else "blue"
            a = g["score"].get(t, 0)
            b = g["score"].get(z, 0)
            scores["won" if a > b else "lost" if a < b else "drawn"] += 1
            mixed.append(dict(seed=g["seed"], trained_color=t, score=[a, b], prefix_samples=prefix))
        else:
            prior = [r for r in ref["recoveries"] if r["kind"] == "takeaway"]
            pairs.append(dict(seed=g["seed"], prefix_samples=prefix, previous=prior, trained=clean))
    summary = summarize([g for g, _ in old + new])
    summary.update(
        pairs=pairs,
        mixed_games=mixed,
        head_to_head_record=scores,
        geometry=geo,
        clean_outcomes=by_kind,
    )
    changes = np.array(
        [
            sum("goal" in r for r in p["trained"]) - sum("goal" in r for r in p["previous"])
            for p in pairs
        ]
    )
    summary["paired_goals_per_match_difference"] = float(changes.mean())
    summary["paired_bootstrap_95"] = np.percentile(
        np.random.default_rng(178).choice(changes, (20000, len(changes))).mean(axis=1), [2.5, 97.5]
    ).tolist()
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("control", type=Path)
    parser.add_argument("trained", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    save(args.output, compare(args.control, args.trained))
