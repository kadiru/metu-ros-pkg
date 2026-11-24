"""Compare two sets of native 5v5 matches that share the same kickoff states."""

import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from audit_five_roles import audit, load_match


def summarize(folder, game):
    report = audit(folder)
    clearance = game["robot_clearance"]
    return dict(
        score=game["score"],
        passes=report["passes"],
        completed_passes=len(report["confirmed_passes"]),
        receptions=report["confirmed_passes"],
        planner_decisions=game["planner_decisions"],
        contact_episodes=clearance["contact_episodes"],
        min_robot_gap_m=clearance["min_gap_m"],
    )


def compare(released, updated):
    pairs = []
    for folder in sorted(released.iterdir()):
        counterpart = updated / folder.name
        if not folder.is_dir():
            continue
        old, before = load_match(folder)
        new, after = load_match(counterpart)
        for key in ("seed", "seconds", "dt", "keys", "sign"):
            if old[key] != new[key]:
                raise ValueError(f"Different {key} for kickoff {folder.name}")
        if not np.array_equal(before[0], after[0]) or (
            old["frames"][0]["opening_velocities"] != new["frames"][0]["opening_velocities"]
        ):
            raise ValueError(f"Different native opening state for kickoff {folder.name}")
        pairs.append(
            dict(
                seed=old["seed"],
                seconds=old["seconds"],
                released=summarize(folder, old),
                updated=summarize(counterpart, new),
            )
        )
    if not pairs or len({pair["seed"] for pair in pairs}) != len(pairs):
        raise ValueError("Use at least one matched kickoff with distinct seeds")
    if len(pairs) != sum(child.is_dir() for child in updated.iterdir()):
        raise ValueError("Both sets must contain the same kickoffs")
    totals = {}
    for arm in ("released", "updated"):
        rows = [pair[arm] for pair in pairs]
        decisions = Counter()
        for row in rows:
            decisions.update(row["planner_decisions"])
        totals[arm] = dict(
            goals=sum(sum(row["score"]) for row in rows),
            passes=sum(row["passes"] for row in rows),
            completed_passes=sum(row["completed_passes"] for row in rows),
            contact_episodes=sum(row["contact_episodes"] for row in rows),
            matches_with_contact=sum(row["contact_episodes"] > 0 for row in rows),
            min_robot_gap_m=min(row["min_robot_gap_m"] for row in rows),
            planner_decisions=dict(decisions),
        )
    return dict(pairs=pairs, totals=totals)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("released", type=Path)
    parser.add_argument("updated", type=Path)
    args = parser.parse_args()
    print(json.dumps(compare(args.released, args.updated), indent=2))
