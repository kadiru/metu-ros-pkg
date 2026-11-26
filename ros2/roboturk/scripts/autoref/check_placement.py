"""Check the preregistered 40-start placement gate against native and official evidence."""

import argparse
from collections import Counter
import json
from pathlib import Path
import statistics

from run_placement import ROOT


def check(report):
    rows = report["trials"]
    p = json.loads((ROOT / "policies/placement_v1/protocol.json").read_text())
    lo, hi = p["official_holdout"]
    if [r["seed"] for r in rows] != list(range(lo, hi + 1)):
        raise ValueError("Reserved start range differs")
    if Counter(r["team"] for r in rows) != {"blue": 20, "yellow": 20}:
        raise ValueError("Unbalanced placement awards")
    if (
        len({tuple(r["initial_ball"]) for r in rows}) != 40
        or len({r["placement_counter"] for r in rows}) != 40
    ):
        raise ValueError("A start or a referee command was reused")
    accepted = []
    seen = set()
    for r in rows:
        if not r["accepted"]:
            continue
        team = r["team"].upper()
        (event,) = [e for e in r["controller_events"] if e["type"] == "PLACEMENT_SUCCEEDED"]
        (observer,) = [e for e in r["observer_events"] if e["type"] == "PLACEMENT_SUCCEEDED"]
        if (
            event["id"] in seen
            or event["id"] not in r["event_ids"]
            or event["placementSucceeded"] != observer["placementSucceeded"]
            or event["placementSucceeded"]["byTeam"] != team
            or "TIGERs AutoRef" not in event["origin"]
            or r["placed_seconds"] > 30
            or r["final_error_m"] > 0.15
            or r["speed_at_acceptance"] > 0.035
            or r["kicks"]
            or not {"capture", "carry", "retreat"}.issubset(phase for _, phase in r["phases"])
        ):
            raise ValueError(f"Inconsistent placement evidence on {r['seed']}")
        seen.add(event["id"])
        accepted.append(r)
    if report["interference"] or len(accepted) < 38 or len(accepted) != report["accepted"]:
        raise ValueError(
            f"Gate failed with {len(accepted)} placements and {report['interference']} interference reports"
        )
    return dict(
        accepted=len(accepted),
        total=len(rows),
        by_team=dict(Counter(r["team"] for r in accepted)),
        interference=report["interference"],
        mean_error_m=statistics.mean(r["final_error_m"] for r in accepted),
        max_error_m=max(r["final_error_m"] for r in accepted),
        mean_acceptance_seconds=statistics.mean(r["placed_seconds"] for r in accepted),
        max_acceptance_seconds=max(r["placed_seconds"] for r in accepted),
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("audit", type=Path)
    print(json.dumps(check(json.loads(parser.parse_args().audit.read_text())), indent=2))
