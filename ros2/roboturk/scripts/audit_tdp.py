#!/usr/bin/env python3
"""Independent 5ms physical-event audit; does not import environment success code."""

import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path


def robot(snapshot, key):
    return next(r for r in snapshot["robots"] if (r["team"], r["id"]) == key)


def mouth(snapshot, key):
    r = robot(snapshot, key)
    x, y, yaw = r["pose"]
    b = snapshot["ball"]
    dx, dy = b[0] - x, b[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    return abs(c * dx + s * dy - 0.115) < 0.035 and abs(-s * dx + c * dy) < 0.045


def crossing(before, after, line, sign):
    a, b = sign * before[0], sign * after[0]
    if not a <= line < b:
        return None
    fraction = (line - a) / (b - a)
    return before[1] + fraction * (after[1] - before[1])


def audit_episode(e):
    assert e["contract"] == "roboturk.tdp_options.v1"
    assert e["backend"] == "webots_ideal_body_velocity"
    assert e["backend_contract"] == "roboturk.webots.closest_kicker.v1"
    key = (e["actor"]["team"], e["actor"]["id"])
    sign = e["attack_sign"]
    samples = e["samples"]
    assert len(samples) == e["physics_checks"] and len(samples) % 4 == 0
    assert abs(len(samples) * 0.005 - e["elapsed_seconds"]) < 1e-7
    kicks = [v for v in e["events"] if v["type"] == "kick"]
    actor = [v for v in kicks if (v["robot"]["team"], v["robot"]["id"]) == key]
    server = [v for v in kicks if v not in actor]
    assert len(actor) == e["actual_actor_kicks"] and len(server) == e["actual_server_kicks"]
    for event in kicks:
        k = (event["robot"]["team"], event["robot"]["id"])
        assert mouth(event["snapshot"], k), "Applied kick outside mouth"
        assert (
            (event["step"] - 1) * 0.02 - 1e-7
            <= event["snapshot"]["time"]
            < event["step"] * 0.02 + 1e-7
        )
        if event in actor:
            assert event["speed"] == 3.5
        else:
            assert (
                k[0] == key[0]
                and k[1] != key[1]
                and 1.6 <= event["speed"] <= 2.5
                and event["step"] == 1
            )
    assert len({round(v["snapshot"]["time"], 8) for v in kicks}) == len(kicks)
    contact = collisions = exits = opponent = 0
    dwell = maximum = 0
    first = None
    goals = own = 0
    valid_clear = False
    premature = False
    previous = e["initial"]["ball"]
    all_held = True
    for i, snapshot in enumerate(samples):
        assert abs(snapshot["time"] - (i + 1) * 0.005) < 1e-7
        r = robot(snapshot, key)
        x, y, _ = r["pose"]
        b = snapshot["ball"]
        held = mouth(snapshot, key)
        contact += held
        all_held &= held
        if held and first is None:
            first = snapshot["time"]
        others = [p for p in snapshot["robots"] if (p["team"], p["id"]) != key]
        collisions += any(math.hypot(x - p["pose"][0], y - p["pose"][1]) < 0.183 for p in others)
        exits += abs(x) > 4.41 or abs(y) > 2.91
        opponent += any(
            p["team"] != key[0] and math.hypot(b[0] - p["pose"][0], b[1] - p["pose"][1]) < 0.116
            for p in others
        )
        gy = crossing(previous, b, 4.5215, sign)
        goals += gy is not None and abs(gy) < 0.4785
        oy = crossing(previous, b, 4.5215, -sign)
        own += oy is not None and abs(oy) < 0.4785
        if e["kind"] == "clear":
            cy = crossing(previous, b, -1, sign)
            if cy is not None:
                side = 1 if sign * e["intent"]["outlet"][1] > 0 else -1
                valid_clear |= 0.8 <= side * sign * cy <= 2.2
        previous = b
        if i % 4 == 0:
            for event in actor:
                if event["step"] == i // 4 + 1 and e["kind"] == "clear":
                    premature |= maximum < 20
        if i % 4 == 3:
            stable = (
                all_held
                and math.hypot(b[2] - r["velocity"][0], b[3] - r["velocity"][1]) < 0.2
                and math.hypot(b[2], b[3]) < 0.25
            )
            dwell = dwell + 1 if stable else 0
            maximum = max(maximum, dwell)
            all_held = True
    if actor:
        first = min([v["snapshot"]["time"] for v in actor] + ([first] if first is not None else []))
    assert (
        contact == e["contact_checks"]
        and collisions == e["collisions"]
        and exits == e["field_exits"]
    )
    assert (
        opponent == e["opponent_touches"] and abs(maximum * 0.02 - e["maximum_stable_dwell"]) < 1e-7
    )
    assert (first is None and e["first_contact_time"] is None) or (
        first is not None and abs(first - e["first_contact_time"]) < 1e-7
    )
    assert bool(premature) == e["premature_clearance"]
    actual_goal_events = [v for v in e["events"] if v["type"] == "goal"]
    assert len(actual_goal_events) == goals
    for event in actual_goal_events:
        y = crossing(event["before"], event["after"], 4.5215, sign)
        assert y is not None and abs(y) < 0.4785
    delay = actor[0]["snapshot"]["time"] - first if actor and first is not None else None
    if e["kind"] == "receive":
        success = dwell >= 25 and not actor and len(server) == 1
    elif e["kind"] == "redirect":
        success = (
            goals == 1
            and len(actor) == 1
            and len(server) == 1
            and delay is not None
            and -0.0051 <= delay <= 0.15
        )
    else:
        success = valid_clear and len(actor) == 1 and not premature
    failed = (
        collisions
        or exits
        or opponent
        or own
        or premature
        or len(actor) > 1
        or ((abs(previous[1]) > 3.05 or abs(previous[0]) > 4.6) and not goals)
    )
    success = bool(success and not failed)
    assert success == e["is_success"], ("Reported outcome differs", e["seed"], success, e["reason"])
    return {
        "seed": e["seed"],
        "kind": e["kind"],
        "success": success,
        "goals": goals,
        "own_goals": own,
        "collisions": int(collisions),
        "field_exits": int(exits),
        "premature_clearance": bool(premature),
        "redirect_delay_seconds": delay if e["kind"] == "redirect" else None,
        "maximum_stable_dwell_seconds": maximum * 0.02,
        "actor_kicks": len(actor),
        "server_kicks": len(server),
        "physics_samples": len(samples),
    }


def audit_file(path, seed=None, count=None):
    path = Path(path)
    raw = path.read_bytes()
    payload = gzip.decompress(raw) if path.suffix == ".gz" else raw
    data = json.loads(payload)
    episodes = data["episodes"]
    if count is not None:
        assert len(episodes) == count
    if seed is not None:
        assert [e["seed"] for e in episodes] == list(range(seed, seed + len(episodes)))
    results = [audit_episode(e) for e in episodes]
    assert sum(r["success"] for r in results) == data["successes"]
    return {
        "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "payload_sha256": hashlib.sha256(payload).hexdigest(),
        "episodes": len(results),
        "successes": sum(r["success"] for r in results),
        "success_rate": sum(r["success"] for r in results) / len(results),
        "collisions": sum(r["collisions"] > 0 for r in results),
        "field_exits": sum(r["field_exits"] > 0 for r in results),
        "own_goals": sum(r["own_goals"] for r in results),
        "premature_clearances": sum(r["premature_clearance"] for r in results),
        "physics_samples": sum(r["physics_samples"] for r in results),
        "cases": results,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("result", type=Path)
    p.add_argument("--seed", type=int)
    p.add_argument("--count", type=int)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    result = audit_file(a.result, a.seed, a.count)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "cases"}, indent=2))


if __name__ == "__main__":
    main()
