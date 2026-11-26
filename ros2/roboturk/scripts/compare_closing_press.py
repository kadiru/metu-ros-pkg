#!/usr/bin/env python3
"""Compare original, fitted and carry-adapted teams on matched closing-marker trials."""

import argparse
from collections import Counter
import json
from pathlib import Path
from statistics import median


def summarize(rows):
    phases, reasons = Counter(), Counter()
    touches, receipts, shots, presses, clashes, steals, losses = 0, 0, 0, 0, 0, 0, 0
    gaps = []
    for row in rows:
        transfer, shot = row["transfer"], row["shooting"]
        last = shot or transfer
        if last["backend"] != "webots_ideal_body_velocity+closing_marker_v1":
            raise ValueError("Expected native Webots closing-marker records")
        if row["is_success"] and not (
            transfer["is_success"]
            and transfer["applied_passes"] == 1
            and shot
            and shot["shot"]
            and shot["applied_kicks"] == 1
            and shot["goal_crossing_y"] is not None
        ):
            raise ValueError("Incomplete sequence in a scored trial")
        presses += last["press_start_seconds"] is not None
        touches += transfer["first_receiver_touch"] is not None
        receipts += transfer["is_success"]
        if transfer["is_success"]:
            gaps.append(transfer["marker_receiver_distance"])
        shots += bool(shot and shot["shot"])
        clashes += last["defender_robot_contacts"] > 0
        steals += last["defender_ball_contacts"] > 0
        losses += bool(transfer["lost_checks"] or (shot and shot["lost_possession_checks"]))
        if row["is_success"]:
            phase = "goal"
        elif shot:
            phase = "after_shot" if shot["shot"] else "carrying"
        elif transfer["first_receiver_touch"]:
            phase = "securing_reception"
        else:
            phase = "pass_in_flight" if transfer["passed"] else "before_valid_pass"
        phases[phase] += 1
        reasons[row["reason"]] += 1
    return dict(
        trials=len(rows),
        valid_passes=sum(r["transfer"]["passed"] for r in rows),
        press_activations=presses,
        first_touches=touches,
        completed_passes=receipts,
        valid_shots=shots,
        goals=phases["goal"],
        marker_robot_proximity_trials=clashes,
        marker_ball_contact_or_possession_trials=steals,
        pre_kick_loss_trials=losses,
        median_marker_gap_at_stable_reception_m=median(gaps) if gaps else None,
        terminal_phases=dict(sorted(phases.items())),
        outcomes=dict(sorted(reasons.items())),
    )


def check_openings(reference_arm, arm):
    rows = reference_arm["episodes"]
    seeds = [r["seed"] for r in rows]
    if not seeds or len(seeds) != len(set(seeds)):
        raise ValueError("Expected unique opening seeds")
    if [r["seed"] for r in arm["episodes"]] != seeds:
        raise ValueError("Different opening seeds")
    for reference, candidate in zip(rows, arm["episodes"], strict=True):
        a, b = reference["transfer"], candidate["transfer"]
        if any(
            a[k] != b[k] for k in ("opening", "defender_speed_limit", "passer_id", "receiver_id")
        ):
            raise ValueError(f"Different opening at {candidate['seed']}")


def timing_comparison(delayed, early):
    check_openings(delayed, early)
    for field in (
        "checkpoint_sha256",
        "checkpoint_manifest_sha256",
        "shooting_checkpoint_sha256",
        "shooting_checkpoint_manifest_sha256",
    ):
        if delayed[field] != early[field]:
            raise ValueError(f"Different frozen policies: {field}")
    base = {k: v for k, v in early["carry_profile"].items() if k != "first_touch_steps"}
    if base != delayed["carry_profile"]:
        raise ValueError("Different carry motion parameters")
    result = {
        "after_hold": summarize(delayed["episodes"]),
        "selected_timing": summarize(early["episodes"]),
    }
    for key in ("receipt", "goal"):
        pairs = Counter()
        for a, b in zip(delayed["episodes"], early["episodes"], strict=True):
            if key == "receipt":
                a, b = a["transfer"], b["transfer"]
            pairs[a["is_success"], b["is_success"]] += 1
        result[key + "_pairs"] = {
            "both": pairs[True, True],
            "selected_only": pairs[False, True],
            "after_hold_only": pairs[True, False],
            "neither": pairs[False, False],
        }
    return result


def compare(original, fitted, adapted):
    arms = {"original": original, "fitted": fitted, "adapted": adapted}
    for key in (
        "checkpoint_sha256",
        "shooting_checkpoint_sha256",
        "shooting_checkpoint_manifest_sha256",
    ):
        if len({d[key] for d in arms.values()}) != 1:
            raise ValueError(f"Different base weights or shooter: {key}")
    if fitted["checkpoint_manifest_sha256"] != adapted[
        "checkpoint_manifest_sha256"
    ] or not adapted.get("carry_profile_sha256"):
        raise ValueError("Expected the same fitted receiver and a recorded carry profile")
    if any(d.get("carry_profile_sha256") for d in (original, fitted)):
        raise ValueError("Both reference arms require the unmodified scorer")
    for arm in arms.values():
        check_openings(original, arm)
    result = {arm: summarize(d["episodes"]) for arm, d in arms.items()}
    # The solver clock and tiny contact differences can vary after unequal previous episode lengths.
    result["transfer_outcome_disagreements"] = [
        a["seed"]
        for a, b in zip(fitted["episodes"], adapted["episodes"], strict=True)
        if a["transfer"]["reason"] != b["transfer"]["reason"]
    ]
    pairs = Counter(
        (a["is_success"], b["is_success"])
        for a, b in zip(fitted["episodes"], adapted["episodes"], strict=True)
    )
    result["paired_carry_goals"] = {
        "both": pairs[True, True],
        "adapted_only": pairs[False, True],
        "fitted_only": pairs[True, False],
        "neither": pairs[False, False],
    }
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("original", "fitted", "adapted"):
        parser.add_argument(key, type=Path)
    parser.add_argument("--timing", type=Path, nargs=2, metavar=("DELAYED", "SELECTED"))
    args = parser.parse_args()
    result = compare(
        *(json.loads(path.read_text()) for path in (args.original, args.fitted, args.adapted))
    )
    if args.timing:
        result["fresh_timing_comparison"] = timing_comparison(
            *(json.loads(path.read_text()) for path in args.timing)
        )
    print(json.dumps(result, indent=2))
