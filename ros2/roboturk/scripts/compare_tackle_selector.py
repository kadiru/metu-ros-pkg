"""Summarize paired native tackle outcomes and check that the selector honored its vetoes."""

import argparse
from collections import Counter
import json
from pathlib import Path
from statistics import median


MODES = ("direct", "pivot_left", "pivot_right", "contain", "selector")
ZONES = ("midfield", "touchline", "own-goal")


def verdict(result):
    return {
        "clean": result["clean_retained"],
        "out": result["ball_out_s"] is not None,
        "goalward": result["goalward_release_s"] is not None,
        "own_goal": result["own_goal_s"] is not None,
        "contact": result["body_contact"],
    }


def compare(paths):
    rows = [row for path in paths for row in json.loads(path.read_text())]
    pairs = {(row["zone"], row["seed"]) for row in rows}
    if not rows or len(rows) != len(pairs):
        raise ValueError("Use unique native start IDs for each zone")
    summaries, policy = {}, {}
    for zone in ZONES:
        group = sorted((r for r in rows if r["zone"] == zone), key=lambda r: r["seed"])
        if not group or any(set(r["outcomes"]) != set(MODES) for r in group):
            raise ValueError(f"Missing paired choices for {zone}")
        summaries[zone] = {}
        for mode in MODES:
            outcomes = [r["outcomes"][mode] for r in group]
            clean = [r["capture_s"] for r in outcomes if r["clean_retained"]]
            measurements = [verdict(r) for r in outcomes]
            summaries[zone][mode] = dict(
                starts=len(group),
                **{k: sum(r[k] for r in measurements) for k in measurements[0]},
                median_clean_capture_s=median(clean) if clean else None,
            )
        events = [r["outcomes"]["selector"]["decisions"] for r in group]
        if any(event["action"] in event["vetoes"] for log in events for event in log):
            raise ValueError(f"The selector used a vetoed action in {zone}")
        policy[zone] = dict(
            first_actions=dict(Counter(log[0]["action"] for log in events)),
            starts_using=dict(Counter(m for log in events for m in {e["action"] for e in log})),
            starts_with_veto=dict(
                Counter(
                    reason
                    for log in events
                    for reason in {
                        reason
                        for e in log
                        for reasons in e["vetoes"].values()
                        for reason in reasons
                    }
                )
            ),
            direct_unsafe_selector_safe=[
                r["seed"]
                for r in group
                if any(v for k, v in verdict(r["outcomes"]["direct"]).items() if k != "clean")
                and not any(
                    v for k, v in verdict(r["outcomes"]["selector"]).items() if k != "clean"
                )
            ],
        )
    return dict(summary=summaries, selector=policy, starts=len(rows))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("panels", type=Path, nargs="+")
    print(json.dumps(compare(parser.parse_args().panels), indent=2))
