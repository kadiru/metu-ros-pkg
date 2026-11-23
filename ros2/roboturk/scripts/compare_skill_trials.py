#!/usr/bin/env python3
"""Audit two native shoot-evaluate or team-evaluate JSON files on identical starts."""

import argparse
from collections import Counter
import json
from pathlib import Path
from statistics import median


def summarize(rows):
    goals = [row for row in rows if row["is_success"]]
    transfers = [row["transfer"] for row in rows if "transfer" in row]
    shots = [row.get("shooting", row) for row in rows if row.get("shooting", row) is not None]
    for row in rows:
        phases = [row.get("transfer"), row.get("shooting", row)]
        if any(p["backend"] != "webots_ideal_body_velocity" for p in phases if p):
            raise ValueError("Expected native Webots records")
        if row["is_success"]:
            shot = row.get("shooting", row)
            if not (
                shot
                and shot["shot"]
                and shot["applied_kicks"] == 1
                and shot["goal_crossing_y"] is not None
            ):
                raise ValueError("Goal is missing an applied shot or measured crossing")
            if "transfer" in row:
                transfer = row["transfer"]
                if not (transfer["received"] and transfer["applied_passes"] == 1):
                    raise ValueError("Goal is missing a completed pass")
    return {
        "trials": len(rows),
        "goals": len(goals),
        "receptions": sum(t["is_success"] for t in transfers) if transfers else None,
        "shooting_episodes": len(shots),
        "valid_shots": sum(s["shot"] for s in shots),
        "possession_loss_trials": sum(
            bool(row.get("transfer", {}).get("lost_checks", 0))
            or bool((row.get("shooting", row) or {}).get("lost_possession_checks", 0))
            for row in rows
        ),
        "robot_collision_trials": sum(r["reason"] == "robot_collision" for r in rows),
        "terminal_reasons": dict(sorted(Counter(row["reason"] for row in rows).items())),
        "median_seconds_to_goal": median(r["elapsed_seconds"] for r in goals) if goals else None,
        "median_seconds_to_receive": median(
            t["elapsed_seconds"] for t in transfers if t["is_success"]
        )
        if any(t["is_success"] for t in transfers)
        else None,
    }


def compare(before, after, *, receiver_changed=False):
    team = "shooter" in before
    hashes = ["checkpoint_sha256"] + (["shooting_checkpoint_sha256"] if team else [])
    if team != ("shooter" in after) or any(before[k] != after[k] for k in hashes):
        raise ValueError("Expected the same task and frozen checkpoints")
    manifests = {}
    for key in hashes:
        field = key.replace("_sha256", "_manifest_sha256")
        old, new = before.get(field), after.get(field)
        if (
            old
            and new
            and old != new
            and not (team and receiver_changed and key == "checkpoint_sha256")
        ):
            raise ValueError(f"Manifest changed: {field}")
        if old or new:
            manifests[field] = [old, new]
    a, b = before["episodes"], after["episodes"]
    if not a or len(a) != len(b) or len({r["seed"] for r in a}) != len(a):
        raise ValueError("Expected equally sized, nonempty cohorts with unique seeds")
    keys = (
        [
            "passer_id",
            "receiver_id",
            "initial_separation",
            "initial_passer_heading",
            "initial_receiver_heading",
        ]
        if team
        else ["moving", "ball_speed", "initial_distance", "initial_heading", "target"]
    )
    changes, times = Counter(), []
    for previous, current in zip(a, b, strict=True):
        x, y = (previous["transfer"], current["transfer"]) if team else (previous, current)
        if previous["seed"] != current["seed"] or any(x[k] != y[k] for k in ["attack_sign", *keys]):
            raise ValueError(f"Opening mismatch at {previous['seed']}")
        changes[
            ("goal" if previous["is_success"] else "miss")
            + "_to_"
            + ("goal" if current["is_success"] else "miss")
        ] += 1
        if previous["is_success"] and current["is_success"]:
            times.append(current["elapsed_seconds"] - previous["elapsed_seconds"])
    return {
        "checkpoint_hashes": {k: before[k] for k in hashes},
        "manifest_hashes_before_after": manifests,
        "old": summarize(a),
        "current": summarize(b),
        "outcome_pairs": dict(sorted(changes.items())),
        "median_paired_change_in_seconds_on_joint_goals": median(times) if times else None,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("current", type=Path)
    parser.add_argument(
        "--receiver-changed",
        action="store_true",
        help="Allow a fitted receiver in the transfer manifest",
    )
    args = parser.parse_args()
    print(
        json.dumps(
            compare(
                json.loads(args.baseline.read_text()),
                json.loads(args.current.read_text()),
                receiver_changed=args.receiver_changed,
            ),
            indent=2,
        )
    )
