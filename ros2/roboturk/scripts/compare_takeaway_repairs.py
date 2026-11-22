"""Measure how much the historical fine-tune transfers at identical fresh kickoffs."""

import argparse
from collections import Counter
from pathlib import Path
import numpy as np
from compare_takeaways import all_games, trace
from train_takeaway import save


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("general", type=Path)
    parser.add_argument("calibrated", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    pairs = []
    regions = {}
    for (a, fa), (b, fb) in zip(all_games(args.general), all_games(args.calibrated), strict=True):
        assert (a["seed"], a["blue_model"], a["yellow_model"], a["initial_world"]) == (
            b["seed"],
            b["blue_model"],
            b["yellow_model"],
            b["initial_world"],
        )
        if b["blue_model"] != b["yellow_model"]:
            continue
        moves = [m for m in b["takeaway_actions"] if m.get("repair_region") is not None]
        sa, sb = trace(fa), trace(fb)
        prefix = 4 * (moves[0]["sample"] // 4 + 1) if moves else len(sa)
        assert np.array_equal(sa[:prefix], sb[:prefix])

        def clean(g):
            return {r["sample"]: r for r in g["recoveries"] if r["kind"] == "takeaway"}

        first, second = clean(a), clean(b)
        for m in moves:
            z = regions.setdefault(m["repair_region"], Counter())
            z["opportunities"] += 1
            z["goals"] += int("goal" in second[m["sample"]])
        record = dict(
            seed=b["seed"],
            verified_prefix_samples=prefix,
            general_goals=sum("goal" in r for r in first.values()),
            calibrated_goals=sum("goal" in r for r in second.values()),
        )
        if moves:
            m = moves[0]
            sample = m["sample"]
            assert first[sample]["winner"] == second[sample]["winner"]
            record["first_repair"] = dict(
                region=m["repair_region"],
                sample=sample,
                general=first[sample]["outcome"],
                calibrated=second[sample]["outcome"],
            )
        pairs.append(record)
    delta = np.array([p["calibrated_goals"] - p["general_goals"] for p in pairs])
    ci = np.percentile(
        np.random.default_rng(181).choice(delta, (20000, len(delta))).mean(axis=1), [2.5, 97.5]
    )
    save(
        args.output,
        dict(
            pairs=pairs,
            regions=regions,
            additional_clean_goals=int(delta.sum()),
            paired_95=ci.tolist(),
            higher=int((delta > 0).sum()),
            equal=int((delta == 0).sum()),
            lower=int((delta < 0).sum()),
        ),
    )


if __name__ == "__main__":
    main()
