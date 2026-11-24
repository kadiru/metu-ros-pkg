"""Compare confirmed clean takeaways, body contacts, and passive AutoRef crash calls."""

import argparse
import json
import math
from pathlib import Path
from statistics import median

from fit_tackle import rows, starts


def audited(trials, events):
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
        result.append(
            dict(
                seed=row["seed"],
                carrier=row["carrier"]["team"],
                motion=("stationary", "forward", "lateral", "turning")[row["seed"] % 4],
                clean_takeaway=row["clean_takeaway"] and challenged_control and crash == 0,
                challenged_control=challenged_control,
                capture_s=row["capture_s"],
                roller_s=row["roller_s"],
                contact=row["contact"],
                first_contact_s=row["first_contact_s"],
                min_gap_m=row["min_gap_m"],
                crash_calls=crash,
            )
        )
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


def compare(old, new, old_events, new_events):
    baseline, candidate = rows(old), rows(new)
    if not baseline or starts(baseline) != starts(candidate):
        raise ValueError("Released and trained branches must use the same native starts")
    if len({r["seed"] for r in baseline}) != len(baseline):
        raise ValueError("Use distinct Webots seeds")
    a, b = audited(baseline, old_events), audited(candidate, new_events)
    totals = dict(current=summarize(a), trained=summarize(b))
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
    args = parser.parse_args()
    print(
        json.dumps(
            compare(args.current, args.trained, args.current_ref, args.trained_ref), indent=2
        )
    )
