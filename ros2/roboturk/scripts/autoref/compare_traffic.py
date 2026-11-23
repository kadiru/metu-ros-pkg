"""Audit matched Webots restarts against raw official outcomes and measured starts."""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics

from run_placement import CRASHES, ROOT


def check_official(row):
    own = row["team"].upper()
    accepted = [e for e in row["controller_events"] if e["type"] == "PLACEMENT_SUCCEEDED"]
    emitted = [e for e in row["observer_events"] if e["type"] == "PLACEMENT_SUCCEEDED"]
    if row["accepted"]:
        assert len(accepted) == len(emitted) == 1
        event = accepted[0]
        assert event["id"] in row["event_ids"] and "TIGERs AutoRef" in event["origin"]
        assert event["placementSucceeded"] == emitted[0]["placementSucceeded"]
        assert event["placementSucceeded"]["byTeam"] == own and row["placed_seconds"] <= 30
        assert row["final_error_m"] < 0.15 and row["speed_at_acceptance"] < 0.035
    crashes = sum(e["type"] in CRASHES for e in row["observer_events"])
    interference = sum(e["type"] == "BOT_INTERFERED_PLACEMENT" for e in row["observer_events"])
    return bool(row["accepted"] and not (crashes or interference))


def compare(old, new, reserved=False, repair=False):
    pairs = list(zip(old["trials"], new["trials"], strict=True))
    assert pairs
    keys = [(r["seed"], r["crossing"]) for r, _ in pairs]
    assert len(set(keys)) == len(keys)
    if reserved or repair:
        name = "repair-protocol.json" if repair else "protocol.json"
        p = json.loads((ROOT / "policies/placement_v2" / name).read_text())
        lo, hi = p["reserved_starts"]
        assert keys == [(s, None) for s in range(lo, hi + 1)]
        assert Counter(r["team"] for r, _ in pairs) == {"blue": 20, "yellow": 20}
    verdicts = []
    maxima = dict(position_m=0.0, yaw_rad=0.0, ball_m=0.0, ball_velocity_mps=0.0)
    if repair:
        maxima["robot_velocity_delta"] = 0.0
    for a, b in pairs:
        assert [a[k] for k in ("seed", "crossing", "team")] == [
            b[k] for k in ("seed", "crossing", "team")
        ]
        for field, key in (
            ("initial_ball", "ball_m"),
            ("opening_ball_velocity", "ball_velocity_mps"),
        ):
            maxima[key] = max(maxima[key], math.dist(a[field], b[field]))
        assert math.dist(a["target"], b["target"]) < 1e-5
        for r, q in zip(a["opening_robots"], b["opening_robots"], strict=True):
            assert r[:2] == q[:2]
            maxima["position_m"] = max(maxima["position_m"], math.dist(r[2:4], q[2:4]))
            maxima["yaw_rad"] = max(
                maxima["yaw_rad"], abs(math.remainder(r[4] - q[4], 2 * math.pi))
            )
        if repair:
            for r, q in zip(
                a["opening_robot_velocities"], b["opening_robot_velocities"], strict=True
            ):
                maxima["robot_velocity_delta"] = max(
                    maxima["robot_velocity_delta"], math.dist(r, q)
                )
        verdicts.append((check_official(a), check_official(b)))

    def summary(report):
        rows = report["trials"]
        accepted = [r for r in rows if r["accepted"]]
        ids = [
            e["id"]
            for r in rows
            for e in r["controller_events"]
            if e["type"] == "PLACEMENT_SUCCEEDED"
        ]
        assert len(set(ids)) == len(ids)
        counts = Counter(e["type"] for r in rows for e in r["observer_events"])
        assert sum(counts[c] for c in CRASHES) == report["crashes"]
        assert counts["BOT_INTERFERED_PLACEMENT"] == report["interference"]
        return dict(
            clean=report["clean"],
            accepted=len(accepted),
            crashes=report["crashes"],
            accepted_crash_calls=report["controller_crashes"],
            interference=report["interference"],
            min_center_m=min(r["min_center_m"] for r in rows),
            episodes_with_centers_under_0183=sum(r["min_center_m"] < 0.183 for r in rows),
            mean_confirmation_s=statistics.mean(r["placed_seconds"] for r in accepted)
            if accepted
            else None,
        )

    result = dict(
        total=len(pairs),
        maximum_opening_deltas=maxima,
        baseline=summary(old),
        candidate=summary(new),
        both=sum(a and b for a, b in verdicts),
        candidate_only=sum(b and not a for a, b in verdicts),
        baseline_only=sum(a and not b for a, b in verdicts),
        neither=sum(not a and not b for a, b in verdicts),
    )
    assert (
        sum(a for a, _ in verdicts) == old["clean"] and sum(b for _, b in verdicts) == new["clean"]
    )
    if repair:
        assert all(v <= 1e-6 for v in maxima.values()), maxima
    if reserved or repair:
        result["gate_passed"] = (
            all(v <= (1e-6 if repair else 1e-5) for v in maxima.values())
            and new["clean"] >= 38
            and new["crashes"] == 0
            and new["interference"] == 0
        )
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--reserved", action="store_true")
    parser.add_argument("--repair", action="store_true")
    args = parser.parse_args()
    result = compare(
        json.loads(args.baseline.read_text()),
        json.loads(args.candidate.read_text()),
        args.reserved,
        args.repair,
    )
    print(json.dumps(result, indent=2))
    if result.get("gate_passed") is False:
        raise SystemExit(1)
