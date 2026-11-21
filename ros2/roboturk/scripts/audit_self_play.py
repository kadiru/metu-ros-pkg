#!/usr/bin/env python3
"""Freeze/audit complete self-play outcomes, including losses and defensive wins."""

import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
SPLITS = {
    "final": (2300000, 200),
    "attack": (2310000, 100),
    "defense": (2320000, 100),
    "demo": (2400000, 8),
}
ROLES = ("defender", "keeper", "transfer", "finish")


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def write(p, data):
    p.write_text(json.dumps(data, indent=2) + "\n")


def wilson(k, n):
    z, p = 1.959963984540054, k / n
    c = (p + z * z / (2 * n)) / (1 + z * z / n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return [max(0, c - h), min(1, c + h)]


def freeze(release):
    if (release / "freeze.json").exists():
        raise ValueError("Refusing to overwrite freeze")
    selection = json.loads((release / "selection.json").read_text())
    for artifact in selection["artifacts"]:
        if digest(release / artifact["path"]) != artifact["sha256"]:
            raise ValueError("Retained training artifact changed")
    paths = (
        sorted(ROOT.glob("src/**/*.py"))
        + sorted(ROOT.glob("src/**/*.proto"))
        + sorted(ROOT.glob("src/**/*.wbt"))
    )
    paths += [
        ROOT / "docs/runtime-manifest.json",
        ROOT / "requirements-macos-validation.txt",
        ROOT / "src/roboturk_ros/webots/controllers/roboturk_supervisor/runtime.ini",
        ROOT / "scripts/run_webots.py",
        ROOT / "scripts/stage_self_play.py",
        Path(__file__).resolve(),
    ]
    models = {}
    for group, roles in selection["models"].items():
        models[group] = {}
        for role, entry in roles.items():
            p = release / entry["path"]
            if (
                digest(p) != entry["sha256"]
                or digest(p.with_suffix(".json")) != entry["manifest_sha256"]
            ):
                raise ValueError("Staged model changed")
            models[group][role] = {**entry, "path": str(p.relative_to(ROOT))}
    result = {
        "created_at": datetime.now(ZoneInfo("America/Los_Angeles")).isoformat(),
        "models": models,
        "runtime_sha256": {str(p.relative_to(ROOT)): digest(p) for p in paths},
        "selection_sha256": digest(release / "selection.json"),
        "splits": SPLITS,
        "gate": {
            "clean_outcomes": 0.95,
            "max_collisions": 0.02,
            "max_unforced_errors": 0.03,
            "min_final_goals": 0.2,
            "min_defensive_stops_vs_bootstrap": 0.2,
        },
    }
    write(release / "freeze.json", result)
    return result


def audit(release, path, split):
    f = json.loads((release / "freeze.json").read_text())
    r = json.loads(path.read_text())
    if path.stat().st_mtime < datetime.fromisoformat(f["created_at"]).timestamp():
        raise ValueError("Result predates freeze")
    if digest(release / "selection.json") != f["selection_sha256"]:
        raise ValueError("Selection changed")
    for artifact in json.loads((release / "selection.json").read_text())["artifacts"]:
        if digest(release / artifact["path"]) != artifact["sha256"]:
            raise ValueError("Retained training artifact changed")
    for p, sha in f["runtime_sha256"].items():
        if digest(ROOT / p) != sha:
            raise ValueError("Frozen runtime changed: " + p)
    used = {}
    for role in ROLES:
        group = (
            "bootstrap"
            if (split == "attack" and role in ("defender", "keeper"))
            or (split == "defense" and role in ("transfer", "finish"))
            else "final"
        )
        model = f["models"][group][role]
        p = ROOT / model["path"]
        if (
            digest(p) != model["sha256"]
            or digest(p.with_suffix(".json")) != model["manifest_sha256"]
        ):
            raise ValueError("Model changed: " + role)
        if r["models"][role]["sha256"] != model["sha256"]:
            raise ValueError("Wrong model: " + role)
        used[role] = model
    if (
        not r["backend"].startswith("webots_")
        or r["opponent_policy_fallback"] is not False
        or r["inference_play"] != "PassReceiveScorePlay"
    ):
        raise ValueError("Not native deployed neural play")
    seed, count = SPLITS[split]
    if [e["seed"] for e in r["episodes"]] != list(range(seed, seed + count)):
        raise ValueError("Missing, duplicated, reordered or unexpected episode")
    cfg = json.loads((ROOT / used["finish"]["path"]).with_suffix(".json").read_text())["config"]
    counters = Counter()
    directions = Counter()
    calls = {role: 0 for role in ("defender", "keeper")}
    excursions = {role: {"x": [], "y": []} for role in calls}
    for e in r["episodes"]:
        t, s = e["transfer"], e["shooting"]
        last = s or t
        if (
            last["opponent"] != "mobile_learned_defender_v1"
            or last["keeper"] != "mobile_learned_keeper_v1"
        ):
            raise ValueError("Wrong defensive controllers")
        total = t["steps"] + (s["steps"] if s else 0)
        if s and not t["is_success"]:
            raise ValueError("Scoring without a completed transfer")
        for role in calls:
            if (
                last[role + "_checkpoint_sha256"] != used[role]["sha256"]
                or last[role + "_policy_calls"] != total
                or last[role + "_physics_checks"] != 4 * total
            ):
                raise ValueError("Opponent inference/physics discontinuity")
            calls[role] += total
            for axis in ("x", "y"):
                excursions[role][axis].append(last[role + "_" + axis + "_excursion"])
        collision = bool(
            last["defender_robot_contacts"]
            or last["keeper_robot_contacts"]
            or "collision" in e["reason"]
        )
        outside = bool(last["defender_outside_checks"] or last["keeper_outside_checks"])
        goal = e["reason"] == "goal"
        stop = e["reason"] in ("defender_interception", "keeper_save")
        if bool(e["is_success"]) != goal:
            raise ValueError("Success disagrees with goal event")
        if goal:
            if not (
                t["passed"]
                and t["received"]
                and t["applied_passes"] == 1
                and t["receive_distance"] >= 1
                and t["receiver_dwell"] >= 20
                and s
                and s["shot"]
                and s["applied_kicks"] == 1
                and s["carried_progress"] >= 0.5
                and s["shot_distance"] <= 1.8
                and s["goal_crossing_y"] is not None
                and abs(s["goal_crossing_y"]) <= cfg["goal_width"] / 2 - cfg["ball_radius"]
                and s["attack_sign"] * s["ball_final"][0] >= cfg["length"] / 2 + cfg["ball_radius"]
            ):
                raise ValueError("Goal lacks required physical events")
            if (
                t["lost_checks"]
                or s["lost_possession_checks"]
                or last["defender_ball_contacts"]
                or last["keeper_ball_contacts"]
            ):
                raise ValueError("Claimed clean goal includes loss/opponent contact")
        if stop:
            role = "defender" if e["reason"] == "defender_interception" else "keeper"
            if last[role + "_ball_contacts"] <= 0:
                raise ValueError("Defensive win lacks contact predicate")
        counters[e["reason"]] += 1
        counters["goals"] += goal
        counters["stops"] += stop and not collision and not outside
        counters["collisions"] += collision
        counters["outside"] += outside
        counters["clean"] += (goal or stop) and not collision and not outside
        counters["unforced"] += not (goal or stop or collision) or outside
        directions[str(t["attack_sign"])] += 1
    if counters["goals"] != r["successes"]:
        raise ValueError("Summary count mismatch")
    gate = (
        counters["clean"] / count >= 0.95
        and counters["collisions"] / count <= 0.02
        and counters["unforced"] / count <= 0.03
    )
    if split == "final":
        gate &= counters["goals"] / count >= 0.2
    if split == "defense":
        gate &= counters["stops"] / count >= 0.2
    for role, axes in excursions.items():
        for axis, values in axes.items():
            values.sort()
            axes[axis] = {
                "min": min(values),
                "median": (values[(count - 1) // 2] + values[count // 2]) / 2,
                "max": max(values),
                "mean": sum(values) / count,
            }
    return {
        "split": split,
        "count": count,
        "passes_gate": bool(gate),
        "counts": dict(counters),
        "directions": dict(directions),
        "policy_calls": calls,
        "excursions_m": excursions,
        "goal_wilson_95": wilson(counters["goals"], count),
        "clean_wilson_95": wilson(counters["clean"], count),
        "models": used,
        "runtime_verified": True,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=["freeze", "audit"])
    p.add_argument("--release", type=Path, required=True)
    p.add_argument("--result", type=Path)
    p.add_argument("--split", choices=SPLITS)
    p.add_argument("--output", type=Path)
    a = p.parse_args()
    result = (
        freeze(a.release.resolve())
        if a.mode == "freeze"
        else audit(a.release.resolve(), a.result, a.split)
    )
    if a.output:
        write(a.output, result)
    print(json.dumps(result, indent=2))
    if not result.get("passes_gate", True):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
