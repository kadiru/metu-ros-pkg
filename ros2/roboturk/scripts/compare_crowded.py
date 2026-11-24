"""Compare native 5v5 outlet arrivals, physical stalls and minimum body gaps."""

import argparse
import json
from pathlib import Path
from statistics import mean, median


def arrival_median(times):
    return median(times) if times else None


def summarize(rows):
    arrivals = [r["arrival_s"] for r in rows if r["arrival_s"] is not None]
    gaps = {k: [r["gaps_m"][k] for r in rows] for k in ("runner", "robot", "board", "goal")}
    return dict(
        starts=len(rows),
        arrivals=len(arrivals),
        timeouts=len(rows) - len(arrivals),
        starts_with_stall=sum(r["stalls"] > 0 for r in rows),
        stall_episodes=sum(r["stalls"] for r in rows),
        total_stalled_s=round(sum(r["stalled_s"] for r in rows), 3),
        median_arrival_among_successes_s=arrival_median(arrivals),
        mean_time_capped_at_deadline_s=mean(
            r["limit_s"] if r["arrival_s"] is None else r["arrival_s"] for r in rows
        ),
        min_gap_m={k: min(v) for k, v in gaps.items()},
        median_trial_min_gap_m={k: median(v) for k, v in gaps.items()},
        starts_with_robot_contact=sum(r["contact_pairs"] > 0 for r in rows),
        starts_with_board_or_goal_contact=sum(
            min(r["gaps_m"]["board"], r["gaps_m"]["goal"]) <= 0.001 for r in rows
        ),
        decisions={
            k: sum(r["decisions"].get(k, 0) for r in rows)
            for k in ("direct", "detour", "errt", "brake")
        },
    )


def compare(previous, current):
    def starts(rows):
        return [(r["seed"], r["initial"], r["runner"], r["target"], r["limit_s"]) for r in rows]

    if (
        not previous
        or starts(previous) != starts(current)
        or len({r["seed"] for r in previous}) != len(previous)
    ):
        raise ValueError(
            "Runs must have identical distinct seeds, native starting states, runners, targets and deadlines"
        )
    paired = [
        (a, b)
        for a, b in zip(previous, current, strict=True)
        if a["arrival_s"] is not None and b["arrival_s"] is not None
    ]
    outcomes = {"both": [], "errt_only": [], "previous_only": [], "neither": []}
    for a, b in zip(previous, current, strict=True):
        key = {
            (True, True): "both",
            (True, False): "previous_only",
            (False, True): "errt_only",
            (False, False): "neither",
        }[a["arrival_s"] is not None, b["arrival_s"] is not None]
        outcomes[key].append(a["seed"])
    return dict(
        current_planner=summarize(previous),
        errt_fallback=summarize(current),
        both_arrived_median_s=[
            arrival_median([p[i]["arrival_s"] for p in paired]) for i in range(2)
        ],
        paired=outcomes,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("current_planner", type=Path)
    p.add_argument("errt_fallback", type=Path)
    args = p.parse_args()
    print(
        json.dumps(
            compare(
                json.loads(args.current_planner.read_text()),
                json.loads(args.errt_fallback.read_text()),
            ),
            indent=2,
        )
    )
