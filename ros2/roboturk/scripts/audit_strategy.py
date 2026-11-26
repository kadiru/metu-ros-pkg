#!/usr/bin/env python3
"""Independent 5 ms geometry audit; never imports the simulator or its metrics.

Mouth possession is a geometric test, not a claim of calibrated roller contact.
Delivery interruption means opponent mouth possession or a subsequent kick.
"""

import argparse
from collections import Counter, defaultdict
import gzip
import hashlib
import json
import math
from pathlib import Path

NAMES = ("blue0", "blue1", "yellow0", "yellow1")
MODES = (
    "recover",
    "dribble",
    "pass",
    "shoot",
    "press",
    "cover",
    "support",
    "receive",
    "redirect",
    "clear",
)
OUTCOMES = ("receive_stop", "redirect_kick", "redirect_goal", "clearance_complete")


def read(path):
    raw = Path(path).read_bytes()
    return json.loads(gzip.decompress(raw) if str(path).endswith(".gz") else raw)


def robot(row, index):
    return row[5 + index * 6 : 11 + index * 6]


def holders(row):
    result = []
    for i in range(4):
        x, y, yaw, *_ = robot(row, i)
        dx, dy = row[1] - x, row[2] - y
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
        if abs(forward - 0.115) < 0.035 and abs(lateral) < 0.045:
            result.append(i)
    return result


def team(index):
    return "blue" if index < 2 else "yellow"


def audit_match(m):
    errors = []

    def require(ok, message):
        if not ok:
            errors.append(message)

    rows, decisions = m["physics_trace"], m["decision_trace"]
    steps = m["steps"]
    require(len(rows) == steps * 4 == m["physics_checks"], "physics trace coverage")
    require(len(decisions) == steps, "decision trace coverage")
    if len(rows) != steps * 4 or len(decisions) != steps:
        return {"seed": m["seed"], "passed": False, "errors": errors}
    require(
        all(len(r) == 29 and all(math.isfinite(v) for v in r) for r in rows),
        "finite complete snapshots",
    )
    events = defaultdict(list)
    for e in m["option_events"]:
        events[e["step"]].append(e)
    base_events = defaultdict(list)
    for e in m["events"]:
        base_events[e["step"]].append(e)
    sign = {"blue": m["attack_sign"], "yellow": -m["attack_sign"]}
    modes = [0] * 4
    states = [None] * 4
    passes, clears = {}, {}
    redirect = None
    derived = []
    counts, motor = Counter(), Counter()
    contacts = outside = contact_events = 0
    in_contact = False
    goals = Counter()
    pending_restart = False
    previous = m["initial_snapshot"]
    last_kick_time = None
    kick_count = 0

    def emit(kind, step, i):
        derived.append((kind, step, NAMES[i]))

    for step, decision in enumerate(decisions, 1):
        require(
            decision[0] == step
            and len(decision) == 5
            and all(isinstance(v, int) and 0 <= v < 10 for v in decision[1:]),
            f"decision indexing {step}",
        )
        restarts = [e for e in events[step - 1] if e["type"] == "restart_snapshot"]
        if restarts:
            require(len(restarts) == 1, f"duplicate restart {step}")
            previous = restarts[0]["snapshot"]
            states, modes, passes, clears, redirect = [None] * 4, [0] * 4, {}, {}, None
            pending_restart = False
        for i, mode in enumerate(decision[1:]):
            pre = [e for e in events[step - 1] if e.get("robot") == NAMES[i]]
            starts = [e for e in pre if e["type"] == "option_start"]
            ends = [e for e in pre if e["type"] == "option_end"]
            expected_end = states[i] is not None and mode != modes[i]
            require(len(ends) == int(expected_end), f"option exit {step}/{i}")
            if mode != modes[i]:
                states[i] = None
            expected_start = mode >= 7 and states[i] is None
            require(len(starts) == int(expected_start), f"option entry {step}/{i}")
            if expected_start and starts:
                require(starts[0]["kind"] == MODES[mode], f"option kind {step}/{i}")
                states[i] = {
                    "dwell": 0,
                    "maximum": 0,
                    "first": None,
                    "notified": False,
                    "intent": starts[0]["intent"],
                }
            for e in pre:
                if e["type"] == "intent_update":
                    require(
                        states[i] is not None and e["kind"] == MODES[mode],
                        f"intent outside active option {step}/{i}",
                    )
                    if states[i] is not None:
                        states[i]["intent"] = e["intent"]
            modes[i] = mode
            counts[team(i) + "/" + MODES[mode]] += 1
            if mode >= 7:
                motor[MODES[mode]] += 1
        motor["continuous_batches"] += 1
        samples = rows[(step - 1) * 4 : step * 4]
        for row in samples:
            require(abs(row[0] - previous[0] - 0.005) < 1e-8, f"physics timing {step}")
            previous = row
        previous = (
            rows[(step - 1) * 4 - 1]
            if step > 1 and not restarts
            else (restarts[0]["snapshot"] if restarts else m["initial_snapshot"])
        )
        applied = [e for e in events[step] if e["type"] == "applied_kick"]
        actual_base = [e for e in base_events[step] if e["type"] in ("pass", "shot")]
        require(len(applied) == len(actual_base), f"kick ledger coverage {step}")
        for e, b in zip(applied, actual_base):
            i = NAMES.index(e["robot"])
            row = e["snapshot"]
            kick_count += 1
            require(e["mode"] == MODES[modes[i]] and e["team"] == team(i), f"kick choice {step}")
            require(i in holders(row), f"kick without mouth possession {step}")
            require(abs(e["speed"] - (2.2 if modes[i] == 2 else 3.5)) < 1e-8, f"kick power {step}")
            require(previous[0] - 1e-8 <= row[0] < samples[-1][0] + 1e-8, f"kick time {step}")
            require(last_kick_time != row[0], f"multiple same-instant kicks {step}")
            last_kick_time = row[0]
            require(
                b["robot"] == e["robot"]
                and abs(b["physics_time"] - row[0]) < 1e-8
                and math.dist(b["ball"], row[1:3]) < 1e-8,
                f"kick ledger mismatch {step}",
            )
            state, t = states[i], team(i)
            if modes[i] in (2, 9):
                passes[t] = {
                    "i": i,
                    "mode": modes[i],
                    "time": row[0],
                    "ball": row[1:3],
                    "step": step,
                }
            passed = passes.get(t)
            if state:
                first = state["first"] if state["first"] is not None else row[0]
                if (
                    modes[i] == 8
                    and passed
                    and passed["mode"] == 2
                    and passed["i"] != i
                    and row[0] - passed["time"] < 5
                    and row[0] - first <= 0.15
                    and math.dist(passed["ball"], row[1:3]) >= 0.5
                ):
                    redirect = {"i": i, "step": step}
                    emit("redirect_kick", step, i)
                elif modes[i] == 9 and state["maximum"] >= 20 and sign[t] * row[1] < -2.2:
                    clears[t] = {"i": i, "step": step, "outlet": state["intent"]["outlet"]}
            if redirect and redirect["step"] != step:
                redirect = None
            clears = {t: c for t, c in clears.items() if c["step"] == step}
        hs = [holders(r) for r in samples]
        for i, state in enumerate(states):
            if state is None:
                continue
            if state["first"] is None:
                state["first"] = next((r[0] for r, h in zip(samples, hs) if i in h), None)
            r = robot(samples[-1], i)
            stable = (
                all(h == [i] for h in hs)
                and math.hypot(samples[-1][3] - r[3], samples[-1][4] - r[4]) < 0.2
                and math.hypot(*samples[-1][3:5]) < 0.25
            )
            state["dwell"] = state["dwell"] + 1 if stable else 0
            state["maximum"] = max(state["maximum"], state["dwell"])
            passed = passes.get(team(i))
            if (
                modes[i] == 7
                and not state["notified"]
                and state["dwell"] >= 25
                and passed
                and passed["mode"] == 2
                and passed["i"] != i
                and samples[-1][0] - passed["time"] < 5
                and math.dist(passed["ball"], samples[-1][1:3]) >= 0.5
            ):
                emit("receive_stop", step, i)
                state["notified"] = True
                del passes[team(i)]
        step_goals = []
        for row, held in zip(samples, hs):
            points = [robot(row, i) for i in range(4)]
            contact = (
                min(math.dist(a[:2], b[:2]) for i, a in enumerate(points) for b in points[i + 1 :])
                < 0.183
            )
            contacts += contact
            contact_events += contact and not in_contact
            in_contact = contact
            outside += any(abs(r[0]) > 4.41 or abs(r[1]) > 2.91 for r in points)
            for t, p in list(passes.items()):
                if row[0] - p["time"] > 5 or any(team(i) != t for i in held):
                    del passes[t]
            for t, c in list(clears.items()):
                if any(team(i) != t for i in held):
                    del clears[t]
                elif sign[t] * previous[1] < -1 <= sign[t] * row[1]:
                    f = (-sign[t] - previous[1]) / (row[1] - previous[1])
                    y = sign[t] * (previous[2] + f * (row[2] - previous[2]))
                    side = 1 if sign[t] * c["outlet"][1] > 0 else -1
                    if 0.8 <= side * y <= 2.2:
                        emit("clearance_complete", step, c["i"])
                    del clears[t]
            if redirect and any(team(i) != team(redirect["i"]) for i in held):
                redirect = None
            for direction in (-1, 1):
                line = direction * 4.5215
                if direction * previous[1] < direction * line <= direction * row[1]:
                    f = (line - previous[1]) / (row[1] - previous[1])
                    y = previous[2] + f * (row[2] - previous[2])
                    if abs(y) < 0.4785 and not pending_restart:
                        t = "blue" if direction == sign["blue"] else "yellow"
                        goals[t] += 1
                        step_goals.append(t)
                        pending_restart = True
            previous = row
        if step_goals and redirect and team(redirect["i"]) in step_goals:
            emit("redirect_goal", step, redirect["i"])
            redirect = None
        reported_goals = [e["team"] for e in base_events[step] if e["type"] == "goal"]
        require(step_goals == reported_goals, f"whole-ball goals {step}")
        if not pending_restart and (abs(previous[1]) > 4.55 or abs(previous[2]) > 3.05):
            pending_restart = True
    reported = [
        (e["type"], e["step"], e["robot"]) for e in m["option_events"] if e["type"] in OUTCOMES
    ]
    require(Counter(derived) == Counter(reported), "skill event geometry/timing")
    require(dict(counts) == m["strategy_ticks"], "strategy inference counts")
    require(dict(motor) == m["motor_calls"], "motor inference counts")
    require(contacts == m["contact_checks"] and contact_events == m["contact_events"], "contacts")
    require(outside == m["outside_checks"], "field exits")
    require(all(goals[t] == m["score"].get(t, 0) for t in sign), "scores")
    return {
        "seed": m["seed"],
        "passed": not errors,
        "errors": errors[:30],
        "physics_samples": len(rows),
        "motor_calls": dict(motor),
        "actual_kicks": kick_count,
        "goals": dict(goals),
        "contact_samples": contacts,
        "outside_samples": outside,
        "skill_outcomes": dict(Counter(e[0] for e in derived)),
    }


def audit(path):
    data = read(path)
    records = [audit_match(m) for m in data["matches"]]
    return {
        "source": str(path),
        "source_sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        "passed": all(r["passed"] for r in records),
        "matches": records,
        "physics_samples": sum(r.get("physics_samples", 0) for r in records),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("input", type=Path)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    result = audit(args.input)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "matches"}))
    for r in result["matches"]:
        if not r["passed"]:
            print(r)
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
