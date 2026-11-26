"""Compare the current selector and joined goalie cover on matched Webots starts."""

import argparse
import json
from pathlib import Path

from compare_tackle_selector import read_rows

ARMS = ("selector", "keeper-cover")
TIMED_ARMS = ("keeper-cover", "keeper-timing")
LANE_ARMS = ("keeper-cover", "keeper-lane")
PIVOT_ARMS = ("keeper-lane", "keeper-pivot")
PANELS = {
    "keeper-cover": ARMS,
    "keeper-timing": TIMED_ARMS,
    "keeper-lane": LANE_ARMS,
    "keeper-pivot": PIVOT_ARMS,
}


def compare(paths):
    rows = sorted((r for p in paths for r in read_rows(p)), key=lambda r: r["seed"])
    if not rows or len({r["seed"] for r in rows}) != len(rows):
        raise ValueError("Use distinct keeper-panel starts")
    zone = rows[0]["zone"]
    arms = PANELS.get(zone)
    if arms is None or any(r["zone"] != zone or set(r["outcomes"]) != set(arms) for r in rows):
        raise ValueError("Expected two matched keeper-cover arms")
    totals, goal_seeds = {}, {}
    for arm in arms:
        samples = [r["outcomes"][arm] for r in rows]
        goal_seeds[arm] = {r["seed"] for r in rows if r["outcomes"][arm]["own_goal_s"] is not None}
        totals[arm] = dict(
            own_goals=len(goal_seeds[arm]),
            clean_takeaways=sum(r["clean_retained"] for r in samples),
            clean_keeper_takeaways=sum(
                r["clean_retained"] and r["captured_by"] == "keeper" for r in samples
            ),
            starts_with_body_contact=sum(r["body_contact"] for r in samples),
            starts_with_cover=sum(r["cover_ticks"] > 0 for r in samples),
            starts_with_early_positioning=sum(r.get("keeper_ready_ticks", 0) > 0 for r in samples),
        )
        if all("body_contact_episodes" in r for r in samples):
            totals[arm]["body_contact_episodes"] = sum(r["body_contact_episodes"] for r in samples)
    a, b = (goal_seeds[k] for k in arms)
    return dict(
        starts=len(rows),
        totals=totals,
        prevented=sorted(a - b),
        introduced=sorted(b - a),
        conceded_by_both=sorted(a & b),
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("panels", type=Path, nargs="+")
    print(json.dumps(compare(parser.parse_args().panels), indent=2))
