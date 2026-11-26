"""Audit paired receptions and identify first-touch losses in native Webots records."""

import argparse
from collections import Counter
import json
from pathlib import Path
from statistics import median

from compare_closing_press import check_openings, summarize


def touch_summary(data):
    rows = data["episodes"]
    summary = summarize(rows)
    groups = {"secured": [], "touched_then_lost": []}
    for row in rows:
        t = row["transfer"]
        if t["first_receiver_touch"]:
            groups["secured" if t["is_success"] else "touched_then_lost"].append(row)
    for group, trials in groups.items():
        gaps, durations, flights = [], [], []
        for row in trials:
            t = row["transfer"]
            touch = t["first_receiver_touch"]
            gaps.append(touch["marker_distance"])
            durations.append(t["elapsed_seconds"] - touch["seconds"])
            dt = t["elapsed_seconds"] / t["steps"]
            flights.append(touch["seconds"] - dt * t["pass_step"])
        summary[group] = {
            "seeds": [r["seed"] for r in trials],
            "transfer_reasons": dict(Counter(r["transfer"]["reason"] for r in trials)),
            "median_marker_gap_at_touch_m": median(gaps) if gaps else None,
            "median_time_touch_to_transfer_end_s": median(durations) if durations else None,
            "median_time_release_to_touch_s": median(flights) if flights else None,
        }
    return summary


def compare(baseline, trained):
    check_openings(baseline, trained)
    for key in (
        "checkpoint_sha256",
        "checkpoint_manifest_sha256",
        "shooting_checkpoint_sha256",
        "shooting_checkpoint_manifest_sha256",
    ):
        if baseline[key] != trained[key]:
            raise ValueError(f"Passer and scorer must match: {key}")
    if {k: v for k, v in trained["carry_profile"].items() if k != "receive"} != baseline[
        "carry_profile"
    ]:
        raise ValueError("Carry profiles must agree after the stable catch")
    result = {"baseline": touch_summary(baseline), "trained": touch_summary(trained)}
    pairs = {}
    labels = {
        (True, True): "both",
        (True, False): "baseline_only",
        (False, True): "trained_only",
        (False, False): "neither",
    }
    for event in ("reception", "goal"):
        groups = {k: [] for k in labels.values()}
        for left, right in zip(baseline["episodes"], trained["episodes"], strict=True):
            a, b = (left["transfer"], right["transfer"]) if event == "reception" else (left, right)
            groups[labels[a["is_success"], b["is_success"]]].append(left["seed"])
        pairs[event] = groups
    flight_deltas, gap_deltas = [], []
    for a, b in zip(baseline["episodes"], trained["episodes"], strict=True):
        left, right = a["transfer"], b["transfer"]
        first, second = left["first_receiver_touch"], right["first_receiver_touch"]
        if first and second:
            release_a = left["pass_step"] * left["elapsed_seconds"] / left["steps"]
            release_b = right["pass_step"] * right["elapsed_seconds"] / right["steps"]
            flight_deltas.append((second["seconds"] - release_b) - (first["seconds"] - release_a))
            gap_deltas.append(second["marker_distance"] - first["marker_distance"])
    result["shared_first_touches"] = {
        "count": len(gap_deltas),
        "median_delta_release_to_touch_s": median(flight_deltas) if flight_deltas else None,
        "median_delta_marker_gap_m": median(gap_deltas) if gap_deltas else None,
    }
    result["paired_seeds"] = pairs
    losses = set(result["baseline"]["touched_then_lost"]["seeds"])
    result["recovered_first_touch_losses"] = sorted(
        losses & set(pairs["reception"]["trained_only"])
    )
    result["goals_from_first_touch_losses"] = sorted(losses & set(pairs["goal"]["trained_only"]))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("trained", type=Path)
    args = parser.parse_args()
    print(
        json.dumps(
            compare(*(json.loads(p.read_text()) for p in (args.baseline, args.trained))), indent=2
        )
    )
