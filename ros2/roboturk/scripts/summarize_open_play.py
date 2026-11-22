"""Compare all paired mirror starts and both color assignments in the mixed games."""

import argparse
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np

from run_recovery_attack import read, save


def count(game, color=None):
    rows = [r for r in game["recoveries"] if color is None or r["team"] == color]
    return dict(recoveries=len(rows), goals=sum(r["outcome"] == "goal" for r in rows))


def compare(folder):
    pairs = defaultdict(dict)
    versus = Counter()
    match_scores = []
    kinds = [Counter(), Counter()]
    failures = [Counter(), Counter()]
    for f in sorted(folder.glob("game-*.json")):
        g = read(f)
        a, b = g["blue_model"], g["yellow_model"]
        if a == b:
            pairs[g["seed"]][a] = count(g)
            for row in g["recoveries"]:
                kinds[a][row["kind"]] += 1
                if row["outcome"] == "goal":
                    kinds[a][row["kind"] + "_goals"] += 1
                    kinds[a]["with_pass"] += bool(row["passes"])
                else:
                    failures[a][row["outcome"]] += 1
        else:
            scores = [g["score"].get(color, 0) for color in ("blue", "yellow")]
            if a == 0:
                scores.reverse()
            versus[
                "won" if scores[0] > scores[1] else "lost" if scores[0] < scores[1] else "drawn"
            ] += 1
            match_scores.append(
                dict(
                    seed=g["seed"],
                    new_color="blue" if a else "yellow",
                    new_score=scores[0],
                    previous_score=scores[1],
                )
            )
    rows = [dict(seed=seed, previous=p[0], trained=p[1]) for seed, p in sorted(pairs.items())]
    differences = np.array([r["trained"]["goals"] - r["previous"]["goals"] for r in rows])
    # Resample starts as units, retaining the two games belonging to each start.
    rng = np.random.default_rng(163)
    draws = rng.choice(differences, size=(20000, len(rows)), replace=True).mean(axis=1)
    return dict(
        pairs=rows,
        mirror_kinds=kinds,
        mirror_misses=failures,
        more_tied_fewer=[
            int((differences > 0).sum()),
            int((differences == 0).sum()),
            int((differences < 0).sum()),
        ],
        extra_goals_per_match=float(differences.mean()),
        paired_bootstrap_95=np.percentile(draws, [2.5, 97.5]).tolist(),
        head_to_head_wdl=versus,
        head_to_head_scores=match_scores,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("matches", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    save(args.output, compare(args.matches))
