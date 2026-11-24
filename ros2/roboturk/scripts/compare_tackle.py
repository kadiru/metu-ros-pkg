"""Compare confirmed clean takeaways, body contacts, and passive AutoRef crash calls."""

import argparse
import json
import math
from pathlib import Path
from statistics import median

from fit_tackle import rows, starts


def roller_sweeps(row):
    """Report turn and ball motion relative to the carrier at release."""
    robots = [r["key"] for r in row["initial"]["robots"]]
    presser, carrier = (4 + 3 * robots.index(row[k]) for k in ("presser", "carrier"))

    def measure(index):
        state = row["trace"][index]
        x, y, yaw = state[carrier : carrier + 3]
        dx, dy = state[0] - x, state[1] - y
        return (
            dx * math.cos(yaw) + dy * math.sin(yaw),
            -dx * math.sin(yaw) + dy * math.cos(yaw),
            state[presser + 2],
        )

    results = []
    for time in row["sweep_times"]:
        start = round(time / 0.04)
        end = min(len(row["control"]), start + round(0.68 / 0.04) + 1)
        released = next((i for i in range(start + 1, end) if not row["control"][i][0]), None)
        depth0, side0, yaw0 = measure(start)
        depth, side, yaw = measure(released if released is not None else end - 1)
        results.append(
            dict(
                start_s=round(time, 3),
                release_s=None if released is None else round(released * 0.04, 3),
                presser_kept_roller=released is not None
                and all(s[1] for s in row["control"][start : released + 1]),
                presser_turn_deg=round(math.degrees(math.remainder(yaw - yaw0, math.tau)), 2),
                ball_lateral_displacement_m=round(side - side0, 4),
                ball_lateral_at_release_m=round(side, 4) if released is not None else None,
                ball_forward_displacement_m=round(depth - depth0, 4),
            )
        )
    return results


def audited(trials, events, duel=False):
    reports = json.loads(events.read_text())
    if [(r["seed"], len(r["trace"])) for r in trials] != [
        (r["seed"], r["frames"]) for r in reports
    ]:
        raise ValueError("AutoRef must adjudicate every recorded native frame of each start")
    result = []
    for row, report in zip(trials, reports, strict=True):
        crash = len(report["crashes"])
        # A loose-ball pickup is not a tackle. Require observed carrier control
        # in the 120 ms before the Presser's first physical-sample roller reach.
        challenged_control = False
        if row["roller_s"] is not None:
            before_reach = math.floor(row["roller_s"] / 0.04 + 1e-8)
            challenged_control = any(
                sample[0] for sample in row["control"][max(0, before_reach - 2) : before_reach + 1]
            )
        trial = dict(
            seed=row["seed"],
            carrier=row["carrier"]["team"],
            motion=("stationary", "reverse" if duel else "forward", "lateral", "turning")[
                row["seed"] % 4
            ],
            clean_takeaway=row["clean_takeaway"] and challenged_control and crash == 0,
            challenged_control=challenged_control,
            capture_s=row["capture_s"],
            roller_s=row["roller_s"],
            contact=row["contact"],
            first_contact_s=row["first_contact_s"],
            min_gap_m=row["min_gap_m"],
            crash_calls=crash,
        )
        if duel:
            trial["sweeps"] = roller_sweeps(row)
        result.append(trial)
    return result


def summarize(trials):
    clean = [r["capture_s"] for r in trials if r["clean_takeaway"]]
    return dict(
        starts=len(trials),
        clean_takeaways=len(clean),
        presser_roller_reaches=sum(r["roller_s"] is not None for r in trials),
        possession_wins_including_contact=sum(r["capture_s"] is not None for r in trials),
        starts_with_body_contact=sum(r["contact"] for r in trials),
        starts_with_ref_crash=sum(r["crash_calls"] > 0 for r in trials),
        ref_crash_calls=sum(r["crash_calls"] for r in trials),
        median_clean_takeaway_s=median(clean) if clean else None,
        min_gap_m=min(r["min_gap_m"] for r in trials),
    )


def compare(old, new, old_events, new_events, duel=False):
    baseline, candidate = rows(old), rows(new)
    if not baseline or starts(baseline) != starts(candidate):
        raise ValueError("Released and trained branches must use the same native starts")
    if len({r["seed"] for r in baseline}) != len(baseline):
        raise ValueError("Use distinct Webots seeds")
    a, b = audited(baseline, old_events, duel), audited(candidate, new_events, duel)
    totals = dict(current=summarize(a), trained=summarize(b))
    if duel:
        for arm, data in (("current", a), ("trained", b)):
            totals[arm]["starts_with_pivot"] = sum(bool(d["sweeps"]) for d in data)
            totals[arm]["pivots_with_lateral_release"] = sum(
                sweep["presser_kept_roller"] and abs(sweep["ball_lateral_at_release_m"]) >= 0.045
                for d in data
                for sweep in d["sweeps"]
            )
    both = [
        x["seed"] for x, y in zip(a, b, strict=True) if x["clean_takeaway"] and y["clean_takeaway"]
    ]
    gained = [
        x["seed"]
        for x, y in zip(a, b, strict=True)
        if not x["clean_takeaway"] and y["clean_takeaway"]
    ]
    lost = [
        x["seed"]
        for x, y in zip(a, b, strict=True)
        if x["clean_takeaway"] and not y["clean_takeaway"]
    ]
    return dict(
        current=a, trained=b, totals=totals, both=both, new_takeaways=gained, lost_takeaways=lost
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("current", "trained", "current_ref", "trained_ref"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--duel", action="store_true", help="Report close-range pivot mechanics")
    args = parser.parse_args()
    print(
        json.dumps(
            compare(args.current, args.trained, args.current_ref, args.trained_ref, args.duel),
            indent=2,
        )
    )
