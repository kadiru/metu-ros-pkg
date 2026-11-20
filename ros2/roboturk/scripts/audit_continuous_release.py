#!/usr/bin/env python3
"""Audit continuous match evidence independently of policy success or score."""

from collections import Counter
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def counterattacks(match):
    """Stable control after a turnover advances >=0.75m within10s before the next loss/restart."""
    successful = []
    events = match["events"]
    for i, event in enumerate(events):
        if event["type"] != "turnover":
            continue
        team, start = event["team"], event["step"]
        sign = match["attack_sign"] * (1 if team == "blue" else -1)
        end = min(match["steps"], start + 500)
        for next_event in events[i + 1 :]:
            if next_event["type"] in ("referee_restart", "ball_out", "goal") or (
                next_event["type"] == "turnover" and next_event["team"] != team
            ):
                end = min(end, next_event["step"])
                break
        best = 0.0
        for row in match["trace"]:
            if start <= row["step"] <= end and any(
                r[0].startswith(team) and r[5] >= 15 for r in row["robots"]
            ):
                best = max(best, sign * (row["ball"][0] - event["ball"][0]))
        if best >= 0.75:
            successful.append(
                {
                    "team": team,
                    "turnover_step": start,
                    "until_step": end,
                    "controlled_advance": best,
                }
            )
    return successful


def audit_match(match, native=True):
    steps = match["steps"]
    assert match["metric_version"] == 2
    assert set(match["calls"]) == {"blue0", "blue1", "yellow0", "yellow1"}
    assert set(match["calls"].values()) == {steps}
    assert sum(match["mode_ticks"].values()) == 4 * steps
    assert match["physics_checks"] == steps * (4 if native else 1)
    assert 0 <= match["contact_checks"] <= match["physics_checks"]
    assert math.isclose(
        match["contact_fraction"], match["contact_checks"] / max(1, match["physics_checks"])
    )
    assert match["backend_resets"] == 1 + match["referee_restarts"]
    assert (
        sum(match["possession_seconds"].values()) + match["contested_seconds"]
        <= match["duration"] + 1e-6
    )
    events = match["events"]
    assert [e["step"] for e in events] == sorted(e["step"] for e in events)
    goals, restarts, passes, shots, receptions, acquisitions, turnovers, recoveries = (
        Counter() for _ in range(8)
    )
    last_owner = None
    last_pass = None
    pending = None
    for e in events:
        typ = e["type"]
        assert 0 < e["step"] <= steps
        if typ == "goal":
            assert pending is None
            sign = e["direction"]
            a, b = e["segment"]
            line = sign * (4.5 + 0.0215)
            assert sign * a[0] < sign * line <= sign * b[0]
            y = a[1] + (line - a[0]) / (b[0] - a[0]) * (b[1] - a[1])
            assert abs(y) < 0.5 - 0.0215 and math.isclose(y, e["crossing_y"], abs_tol=1e-9)
            assert e["team"] == ("blue" if sign == match["attack_sign"] else "yellow")
            goals[e["team"]] += 1
            pending = ("goal", e["step"])
        elif typ == "ball_out":
            assert pending is None
            pending = ("ball_out", e["step"])
        elif typ == "referee_restart":
            assert pending == (e["reason"], e["step"])
            pending = None
            restarts[e["reason"]] += 1
            last_owner = last_pass = None
        elif typ in ("pass", "shot"):
            assert e["held"] and e["robot"].startswith(e["team"])
            assert e["mode"] == ("pass" if typ == "pass" else "shoot")
            assert math.isclose(e["speed"], 2.2 if typ == "pass" else 3.5)
            (passes if typ == "pass" else shots)[e["team"]] += 1
            last_pass = e if typ == "pass" else None
        elif typ in ("acquisition", "turnover"):
            team = e["team"]
            assert e["dwell"] >= 15 and e["previous_robot"] == last_owner
            is_turnover = last_owner is not None and not last_owner.startswith(team)
            assert (typ == "turnover") == is_turnover
            acquisitions[team] += 1
            turnovers[team] += int(is_turnover)
            recoveries[team] += int(e["after_loose_ball"])
            if last_pass and (last_pass["team"] != team or last_pass["robot"] == e["robot"]):
                last_pass = None
            last_owner = e["robot"]
        elif typ == "reception":
            assert last_pass is not None
            assert last_pass["step"] == e["pass_step"]
            assert last_pass["team"] == e["team"] and last_pass["robot"] != e["robot"]
            assert 0 < e["step"] - last_pass["step"] <= 250
            assert e["dwell"] >= 15 and e["displacement"] >= 0.5
            assert math.isclose(
                math.dist(last_pass["ball"], e["ball"]), e["displacement"], abs_tol=1e-9
            )
            receptions[e["team"]] += 1
            last_pass = None
        else:
            raise AssertionError("Unknown event " + typ)
    for key, counts in [
        ("score", goals),
        ("passes", passes),
        ("shots", shots),
        ("receptions", receptions),
        ("acquisitions", acquisitions),
        ("turnovers", turnovers),
        ("recoveries", recoveries),
    ]:
        assert all(match[key].get(t, 0) == counts[t] for t in ("blue", "yellow")), key
    assert sum(restarts.values()) == match["referee_restarts"]
    if pending:
        assert pending[1] == steps
    assert match["trace"] and match["trace"][-1]["step"] == steps
    assert all(len(row["robots"]) == 4 for row in match["trace"])
    return counterattacks(match)


def audit_file(path, expected_seed=None, expected_count=None):
    result = json.loads(Path(path).read_text())
    assert result["backend"] == "webots_ideal_body_velocity" and result["control"] == "neural"
    assert result["contact_terminates"] is False and result["runtime_action_projection"] is False
    matches = result["matches"]
    if expected_count is not None:
        assert len(matches) == expected_count
    if expected_seed is not None:
        assert [m["seed"] for m in matches] == list(
            range(expected_seed, expected_seed + len(matches))
        )
    attacks = []
    for m in matches:
        attacks.extend({"seed": m["seed"], **event} for event in audit_match(m))
    from roboturk.continuous import summary

    expected = summary(matches)
    if "unfinished" not in result["summary"]:
        # Retain the old development reports as written, while preventing their
        # score ties in aborted games from being treated as qualified draws.
        expected.pop("unfinished")
        expected["wins"] = {
            t: sum(m["score"][t] > m["score"]["yellow" if t == "blue" else "blue"] for m in matches)
            for t in ("blue", "yellow")
        }
        expected["draws"] = sum(m["score"]["blue"] == m["score"]["yellow"] for m in matches)
    assert result["summary"] == expected
    return {
        "matches": len(matches),
        "summary": result["summary"],
        "counterattacks": attacks,
        "counterattack_matches": len({e["seed"] for e in attacks}),
        "source_sha256": digest(path),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("result", type=Path)
    p.add_argument("--seed", type=int)
    p.add_argument("--count", type=int)
    p.add_argument("--output", type=Path)
    args = p.parse_args()
    result = audit_file(args.result, args.seed, args.count)
    if args.output:
        args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
