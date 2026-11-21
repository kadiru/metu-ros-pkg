#!/usr/bin/env python3
"""Compare the changed side on matched native development starts."""

import argparse
import json
from pathlib import Path


def compare(trained, random):
    counts = {k: {"passes": 0, "complete": 0, "clearer_lane": 0} for k in ("trained", "random")}
    pairs = []
    for index, trained_path in enumerate(sorted(trained.glob("game-*.json"))):
        team = "yellow" if index % 2 else "blue"
        row = {"team": team}
        for name, path in [("trained", trained_path), ("random", random / trained_path.name)]:
            match = json.loads(path.read_text())
            assert row.get("seed", match["seed"]) == match["seed"]
            row["seed"] = match["seed"]
            kicks = [p for p in match["passes"] if p["team"] == team]
            stats = {
                "passes": len(kicks),
                "complete": sum(p["result"] == "complete" for p in kicks),
                "clearer_lane": sum(
                    p["features"][p["slot"]][0] > p["features"][1 - p["slot"]][0] for p in kicks
                ),
            }
            for key, value in stats.items():
                counts[name][key] += value
            row[name] = stats
        pairs.append(row)
    assert len(pairs) == len(list(random.glob("game-*.json"))) and pairs
    return {
        "sides": counts,
        "cases": pairs,
        "scope": "Counts from the side whose receiver policy changed on matched starts. Play trajectories can diverge.",
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("trained", type=Path)
    p.add_argument("random", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    summary = compare(a.trained, a.random)
    a.output.write_text(json.dumps(summary, indent=2) + "\n")
    print(summary["sides"])


if __name__ == "__main__":
    main()
