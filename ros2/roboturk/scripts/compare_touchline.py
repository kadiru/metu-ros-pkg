"""Compare native six-second touchline trials, rejecting unpaired or duplicate starts."""

import argparse
import json
from pathlib import Path
from statistics import median


def summarize(rows):
    recovered = [r["recovered"] for r in rows if r["recovered"]]
    return {
        "openings": len(rows),
        "recoveries": len(recovered),
        "recovery_teams": {t: sum(r["team"] == t for r in recovered) for t in ("blue", "yellow")},
        "median_recovery_seconds": median(r["seconds"] for r in recovered) if recovered else None,
        "crossed_live": sum(r["crossed"] for r in rows),
        "sole_control_outside": sum(r["outside_hold"] for r in rows),
        "ball_out_before_recovery": sum(r["end"] == "out" and not r["recovered"] for r in rows),
        "contacts": {k: sum(r["contacts"][k] for r in rows) for k in ("pair", "board", "goal")},
        "minimum_clearance_m": {
            k: min(r["min_gap_m"][k] for r in rows) for k in ("pair", "board", "goal")
        },
    }


def compare(previous, current):
    def starts(rows):
        return [(r["seed"], r["start"], r["initial"]) for r in rows]

    if (
        not previous
        or starts(previous) != starts(current)
        or len({r["seed"] for r in current}) != len(current)
    ):
        raise ValueError(
            "Expected unique seeds and identical native reset positions and velocities in the same order"
        )
    outcomes = {k: [] for k in ("both", "previous_only", "current_only", "neither")}
    for a, b in zip(previous, current, strict=True):
        label = (
            ("both" if a["recovered"] else "current_only")
            if b["recovered"]
            else ("previous_only" if a["recovered"] else "neither")
        )
        outcomes[label].append(b["seed"])
    return dict(previous=summarize(previous), current=summarize(current), outcomes=outcomes)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("previous", type=Path)
    parser.add_argument("current", type=Path)
    args = parser.parse_args()
    print(
        json.dumps(
            compare(json.loads(args.previous.read_text()), json.loads(args.current.read_text())),
            indent=2,
        )
    )
