#!/usr/bin/env python3
"""Audit frozen reserved-seed evidence without silently filtering failures."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def audit(result, seed, count, kind):
    records = result["episodes"]
    if [r["seed"] for r in records] != list(range(seed, seed + count)):
        raise ValueError("Expected every reserved seed exactly once, in order")
    successes = sum(bool(r["is_success"]) for r in records)
    if successes != result["successes"]:
        raise ValueError("Reported success count disagrees with episode records")
    errors = Counter(r["reason"] for r in records if not r["is_success"])
    losses = collisions = 0
    for r in records:
        if kind == "play":
            transfer, shot = r["transfer"], r["shooting"]
            losses += bool(transfer["lost_checks"] or (shot and shot["lost_possession_checks"]))
            collisions += bool(
                transfer.get("defender_robot_contacts", 0)
                or transfer.get("keeper_robot_contacts", 0)
                or (
                    shot
                    and (
                        shot.get("defender_robot_contacts", 0)
                        or shot.get("keeper_robot_contacts", 0)
                    )
                )
                or "collision" in r["reason"]
            )
            if r["is_success"]:
                if not (
                    transfer["passed"]
                    and transfer["received"]
                    and transfer["applied_passes"] == 1
                    and transfer["receive_distance"] >= 1.0
                    and transfer["receiver_dwell"] >= 20
                    and shot
                    and shot["shot"]
                    and shot["applied_kicks"] == 1
                    and shot["carried_progress"] >= 0.5
                    and shot["shot_distance"] <= 1.8
                ):
                    raise ValueError("Claimed goal lacks required physical events")
                if shot.get("keeper_ball_contacts", 0) or shot.get("defender_ball_contacts", 0):
                    raise ValueError("Claimed clean goal includes an opponent touch")
        else:
            collisions += "collision" in r["reason"]
            if r["is_success"] and r["stable_dwell_seconds"] < (0.5 if kind == "block" else 3.0):
                raise ValueError("Micro success lacks the required continuous dwell")
    return {
        "count": count,
        "successes": successes,
        "failures": dict(errors),
        "loss_episodes": losses,
        "collision_episodes": collisions,
        "passes_gate": successes / count >= 0.95 and losses == 0 and collisions == 0,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--count", type=int, default=200)
    parser.add_argument("--kind", choices=["play", "block", "cover"], required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--freeze", type=Path, required=True)
    args = parser.parse_args()
    freeze = json.loads(args.freeze.read_text())
    digest = hashlib.sha256(args.checkpoint.read_bytes()).hexdigest()
    if freeze["checkpoint_sha256"] != digest:
        raise ValueError("Checkpoint changed after freeze")
    if args.result.stat().st_mtime < args.freeze.stat().st_mtime:
        raise ValueError("Result predates freeze")
    result = audit(json.loads(args.result.read_text()), args.seed, args.count, args.kind)
    result["checkpoint_sha256"] = digest
    print(json.dumps(result, indent=2))
    if not result["passes_gate"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
