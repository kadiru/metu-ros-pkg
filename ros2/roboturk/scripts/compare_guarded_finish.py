"""Count physical receptions, valid shots, saves and goals from paired guarded trials."""

import argparse
from collections import Counter
import json
from pathlib import Path

from compare_closing_press import check_openings


def summarize(data):
    receipts = shots = saves = goals = 0
    reasons = Counter()
    for r in data["episodes"]:
        t, s = r["transfer"], r["shooting"]
        last = s or t
        if (
            t["task"] != "pass_receive.closing_marker_learned_keeper.v1"
            or t["backend"] != "webots_ideal_body_velocity+closing_marker_v1+learned_keeper_v1"
            or len(t["opening"]["robots"]) != 4
            or last["keeper_policy_calls"] != last["keeper_physics_checks"] // 10
            or last["keeper_physics_checks"] != last["defender_physics_checks"]
        ):
            raise ValueError("Expected two opponents continuously active in native Webots")
        if bool(s) != t["is_success"] or (
            s and (t["receiver_dwell"] < 20 or t["applied_passes"] != 1)
        ):
            raise ValueError("Invalid reception handoff")
        if r["is_success"] and not (
            s
            and s["shot"]
            and s["applied_kicks"] == 1
            and s["goal_crossing_y"] is not None
            and abs(s["goal_crossing_y"]) < 0.5
        ):
            raise ValueError("Missing physical goal evidence")
        receipts += t["is_success"]
        shots += bool(s and s["shot"])
        saves += bool(s and s["shot"] and r["reason"] == "keeper_save")
        goals += r["is_success"]
        reasons[r["reason"]] += 1
    return dict(
        starts=len(data["episodes"]),
        receptions=receipts,
        shots=shots,
        saves_on_shots=saves,
        goals=goals,
        outcomes=dict(sorted(reasons.items())),
    )


def compare(reference, challenger):
    check_openings(reference, challenger)
    if reference["carry_profile"] != {
        k: v for k, v in challenger["carry_profile"].items() if k != "finish"
    }:
        raise ValueError("Receiver or escape carry differs")
    for key in (
        "checkpoint_sha256",
        "checkpoint_manifest_sha256",
        "shooting_checkpoint_sha256",
        "shooting_checkpoint_manifest_sha256",
    ):
        if reference[key] != challenger[key]:
            raise ValueError(f"Different frozen checkpoint at {key}")
    result = {"previous": summarize(reference), "trained": summarize(challenger)}
    pairs = {"both": [], "trained_only": [], "previous_only": [], "neither": []}
    converted, changes = [], []
    for a, b in zip(reference["episodes"], challenger["episodes"], strict=True):
        ta, tb = a["transfer"], b["transfer"]
        if any(
            ta[k] != tb[k]
            for k in (
                "keeper_speed_limit",
                "keeper_style",
                "keeper_reaction_seconds",
                "keeper_acceleration",
                "keeper_checkpoint_sha256",
            )
        ):
            raise ValueError(f"Different goalkeeper at {a['seed']}")
        label = (
            ("both" if a["is_success"] else "trained_only")
            if b["is_success"]
            else ("previous_only" if a["is_success"] else "neither")
        )
        pairs[label].append(a["seed"])
        if ta["is_success"] != tb["is_success"]:
            changes.append(a["seed"])
        if (
            a["reason"] == "keeper_save"
            and a["shooting"]
            and a["shooting"]["shot"]
            and b["is_success"]
        ):
            converted.append(a["seed"])
    return dict(
        **result, goal_pairs=pairs, receipt_disagreements=changes, saved_shots_converted=converted
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("previous", type=Path)
    p.add_argument("trained", type=Path)
    args = p.parse_args()
    print(
        json.dumps(
            compare(*(json.loads(f.read_text()) for f in (args.previous, args.trained))), indent=2
        )
    )
