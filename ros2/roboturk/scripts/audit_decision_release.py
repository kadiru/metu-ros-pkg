#!/usr/bin/env python3
"""Stage, freeze and audit learned tactical decisions without selecting outcomes."""

import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import shutil
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
ROLES = ("defender", "keeper", "manager", "transfer", "finish")
SPLITS = {
    "final": (3300000, 240),
    "attack": (3310000, 120),
    "defense": (3320000, 120),
    "demo": (3400000, 9),
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")


def stage(source, release):
    source, release = Path(source).resolve(), Path(release).resolve()
    if (release / "freeze.json").exists():
        raise ValueError("Refusing to restage frozen release")
    league = json.loads((source / "league.json").read_text())
    history = league["snapshots"]
    updates = league["updates"]
    expected = {(g, r) for g in range(1, len(history)) for r in ("defender", "keeper", "manager")}
    if {(r["generation"], r["role"]) for r in updates} != expected or len(updates) != len(expected):
        raise ValueError("Incomplete role/generation ledger")
    total = accepted = 0
    expected_actors = {r: m["actor_sha256"] for r, m in history[0].items()}
    for bundle in history:
        for role, m in bundle.items():
            if sha(Path(m["path"])) != m["sha256"]:
                raise ValueError("Historical checkpoint changed")
            if (
                role in ("transfer", "finish")
                and m["actor_sha256"] != history[0][role]["actor_sha256"]
            ):
                raise ValueError("Frozen motion actor changed")
    for row in sorted(
        updates, key=lambda r: (r["generation"], ("defender", "keeper", "manager").index(r["role"]))
    ):
        role = row["role"]
        if row["historical_pool"] != history[: row["generation"]]:
            raise ValueError("Historical opponent pool changed")
        if {r: m["actor_sha256"] for r, m in row["before"].items()} != expected_actors:
            raise ValueError("Unreported change between role updates")
        base = source / f"round{row['generation']}" / role
        events = json.loads((base / "updates.json").read_text())
        rolls = json.loads((base / "rollouts.json").read_text())
        resets = json.loads((base / "resets.json").read_text())
        pool_size = len(row["historical_pool"]) + int(role == "manager")
        if (
            len(resets) != row["reset_count"]
            or not resets
            or any(not 3100000 <= r["seed"] < 3200000 for r in resets)
            or any(not 0 <= r["pool_index"] < pool_size for r in resets)
            or [r["seed"] for r in resets[: len(rolls)]] != [r["seed"] for r in rolls]
            or [r["pool_index"] for r in resets[: len(rolls)]]
            != [r["opponent_pool_index"] for r in rolls]
        ):
            raise ValueError("Incomplete or contaminated reset ledger")
        if role == "manager" and (
            sum(row["sampled_manager_choices"].values()) != row["steps"]
            or row["choices_different_from_starting_manager"] <= 0
        ):
            raise ValueError("Manager did not explore decisions")
        if role == "manager" and row.get("precision"):
            from stable_baselines3 import PPO
            from roboturk.decision_precision import coordinate_hash

            hashes = [
                coordinate_hash(PPO.load(row[k][role]["path"], device="cpu"))
                for k in ("before", "after")
            ]
            if hashes[0] != hashes[1] or hashes != [
                row["coordinate_function_before"],
                row["coordinate_function_after"],
            ]:
                raise ValueError("Learned coordinate function changed during tactical PPO")
        if (
            row["steps"] != 32768
            or len(events) != 32
            or row["accepted_updates"] != sum(not e["rejected"] for e in events)
        ):
            raise ValueError("PPO update ledger mismatch")
        if (
            row["actor_before"] == row["actor_after"]
            or row["before"][role]["actor_sha256"] != row["actor_before"]
            or row["after"][role]["actor_sha256"] != row["actor_after"]
        ):
            raise ValueError("Actor update lineage mismatch")
        for r, m in row["after"].items():
            if sha(Path(m["path"])) != m["sha256"]:
                raise ValueError("Updated checkpoint changed")
            if r != role and m["actor_sha256"] != row["before"][r]["actor_sha256"]:
                raise ValueError("Unreported actor update")
        if any(not 3100000 <= e["seed"] < 3200000 for e in rolls):
            raise ValueError("Reserved reset entered training")
        total += row["steps"]
        accepted += row["accepted_updates"]
        expected_actors = {r: m["actor_sha256"] for r, m in row["after"].items()}
        if role == "manager" and expected_actors != {
            r: m["actor_sha256"] for r, m in history[row["generation"]].items()
        }:
            raise ValueError("Generation snapshot differs from accepted actors")
    copied = []
    for p in sorted(source.rglob("*")):
        if p.suffix not in (".json", ".zip") or any(
            x in p.name for x in ("publication", "upload", "verification-manifest")
        ):
            continue
        destination = release / "artifacts" / p.relative_to(source)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, destination)
        copied.append({"path": str(destination.relative_to(release)), "sha256": sha(destination)})
    models = {}
    for group, bundle in [("bootstrap", history[0]), ("final", history[-1])]:
        models[group] = {}
        portable = {}
        for role, m in bundle.items():
            target = release / group / role / "policy.zip"
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(m["path"], target)
            shutil.copy2(Path(m["path"]).with_suffix(".json"), target.with_suffix(".json"))
            models[group][role] = {
                "path": str(target.relative_to(release)),
                "sha256": sha(target),
                "manifest_sha256": sha(target.with_suffix(".json")),
                "actor_sha256": m["actor_sha256"],
            }
            portable[role] = {**models[group][role], "path": str(target.relative_to(ROOT))}
        write(release / (group + "-bundle.json"), portable)
    write(
        release / "training-audit.json",
        {
            "generations": len(history) - 1,
            "role_jobs": len(updates),
            "ppo_steps": total,
            "accepted_updates": accepted,
            "reserved_training_resets": 0,
            "historical_hashes_verified": True,
            "frozen_motion_actor_hashes_verified": True,
        },
    )
    copied.append({"path": "training-audit.json", "sha256": sha(release / "training-audit.json")})
    write(
        release / "selection.json",
        {
            "rule": "last completed generation after development; no reserved selection",
            "generations": len(history) - 1,
            "models": models,
            "artifacts": copied,
        },
    )
    return {
        "models": len(models) * len(ROLES),
        "artifacts": len(copied),
        "ppo_steps": total,
        "accepted_updates": accepted,
    }


def freeze(release):
    if (release / "freeze.json").exists():
        raise ValueError("Refusing to overwrite freeze")
    selection = json.loads((release / "selection.json").read_text())
    for a in selection["artifacts"]:
        if sha(release / a["path"]) != a["sha256"]:
            raise ValueError("Retained artifact changed")
    files = [p for p in ROOT.glob("src/**/*") if p.suffix in (".py", ".proto", ".wbt", ".ini")]
    files += [
        ROOT / "scripts/run_webots.py",
        Path(__file__).resolve(),
        ROOT / "docs/runtime-manifest.json",
        ROOT / "requirements-macos-validation.txt",
    ]
    result = {
        "created_at": datetime.now(ZoneInfo("America/Los_Angeles")).isoformat(),
        "selection_sha256": sha(release / "selection.json"),
        "models": selection["models"],
        "runtime_sha256": {str(p.relative_to(ROOT)): sha(p) for p in sorted(files)},
        "splits": SPLITS,
        "gates": {
            "min_clean": 0.95,
            "max_collisions": 0.02,
            "max_unforced": 0.03,
            "min_final_goals": 0.2,
            "min_final_family_clean": 0.9,
            "min_final_family_goals": 0.1,
            "min_defense_stops": 0.2,
            "min_final_passes": 20,
            "min_final_receptions": 20,
            "min_final_carry_episodes": 20,
            "min_final_shots": 20,
            "min_final_direct_goals": 5,
            "demo_clean": 9,
        },
    }
    write(release / "freeze.json", result)
    return result


def validate_episode(e, cfg, models, require_gate=False):
    steps = e["steps"]
    if e["manager_calls"] != steps or sum(e["option_counts"].values()) != steps:
        raise ValueError("Manager decision discontinuity")
    for role in ("defender", "keeper"):
        if (
            e[role + "_checkpoint_sha256"] != models[role]["sha256"]
            or e[role + "_policy_calls"] != steps
            or e[role + "_physics_checks"] != steps * 4
        ):
            raise ValueError("Opponent inference/physics discontinuity")
    trace = e["decisions"]
    if (
        not trace
        or trace[0]["step"] != 0
        or any(
            b["step"] <= a["step"] or b["step"] - a["step"] > 10 for a, b in zip(trace, trace[1:])
        )
    ):
        raise ValueError("Missing learned decisions")
    previous_target = [0, 0]
    for t in trace:
        if (
            t["option"] not in ("pass", "dribble", "shoot")
            or t["owner"] not in (0, 1)
            or not all(math.isfinite(v) for key in ("receive", "parking", "aim") for v in t[key])
        ):
            raise ValueError("Invalid learned decision")
        if require_gate:
            if type(t.get("receive_update")) is not bool:
                raise ValueError("Missing target update decision")
            if not t["receive_update"] and t["receive"] != previous_target:
                raise ValueError("Target changed without learned update")
            previous_target = t["receive"]
    for event in e["events"]:
        decision = next((t for t in reversed(trace) if t["step"] <= event["step"] - 1), None)
        if (
            decision is None
            or decision["option"] != event["type"]
            or decision["owner"] != event["owner"]
            or event["decision_owner"] != event["owner"]
        ):
            raise ValueError("Applied kick disagrees with learned decision")
        if (event["type"] not in ("pass", "shoot") or not event["held"]) and e["reason"] in (
            "goal",
            "keeper_save",
            "defender_interception",
        ):
            raise ValueError("Unheld/invalid kick event")
        if not math.isclose(event["speed"], 2.5 if event["type"] == "pass" else 3.5, abs_tol=1e-6):
            raise ValueError("Unexpected kick speed")
        if (
            event["type"] == "shoot"
            and event["goal_distance"] > 2
            and e["reason"] in ("goal", "keeper_save", "defender_interception")
        ):
            raise ValueError("Invalid shot range")
    if len(e["reception_events"]) != e["stable_receptions"]:
        raise ValueError("Reception count mismatch")
    for event in e["reception_events"]:
        if (
            event["dwell"] < 20
            or event["pass_displacement"] < 0.5
            or not any(
                k["type"] == "pass" and k["owner"] != event["owner"] and k["step"] < event["step"]
                for k in e["events"]
            )
        ):
            raise ValueError("Reception lacks pass/stable control")
    collision = bool(
        "collision" in e["reason"] or e["defender_robot_contacts"] or e["keeper_robot_contacts"]
    )
    outside = bool(e["defender_outside_checks"] or e["keeper_outside_checks"])
    goal = e["reason"] == "goal"
    stop = e["reason"] in ("defender_interception", "keeper_save")
    if e["is_success"] != goal:
        raise ValueError("Goal and success disagree")
    if goal:
        if (
            not e["events"]
            or e["events"][-1]["type"] != "shoot"
            or e["lost_possession_checks"]
            or e["defender_ball_contacts"]
            or e["keeper_ball_contacts"]
            or collision
            or outside
        ):
            raise ValueError("Goal lacks a clean shot")
        segment = e["goal_crossing_segment"]
        sign = e["attack_sign"]
        line = sign * (cfg["length"] / 2 + cfg["ball_radius"])
        if not segment or not sign * segment[0][0] < sign * line <= sign * segment[1][0]:
            raise ValueError("Goal lacks whole-ball crossing")
        alpha = (line - segment[0][0]) / (segment[1][0] - segment[0][0])
        y = segment[0][1] + alpha * (segment[1][1] - segment[0][1])
        if abs(y) > cfg["goal_width"] / 2 - cfg["ball_radius"] or not math.isclose(
            y, e["goal_crossing_y"], abs_tol=1e-7
        ):
            raise ValueError("Goal misses posts")
    if stop:
        role = "keeper" if e["reason"] == "keeper_save" else "defender"
        if e[role + "_ball_contacts"] <= 0:
            raise ValueError("Stop lacks contact predicate")
    return goal, stop and not collision and not outside, collision, outside


def wilson(k, n):
    z = 1.959963984540054
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return [max(0, c - h), min(1, c + h)]


def audit(release, path, split):
    f = json.loads((release / "freeze.json").read_text())
    result = json.loads(path.read_text())
    if path.stat().st_mtime < datetime.fromisoformat(f["created_at"]).timestamp():
        raise ValueError("Result predates freeze")
    if sha(release / "selection.json") != f["selection_sha256"]:
        raise ValueError("Selection changed")
    for a in json.loads((release / "selection.json").read_text())["artifacts"]:
        if sha(release / a["path"]) != a["sha256"]:
            raise ValueError("Retained artifact changed")
    for p, h in f["runtime_sha256"].items():
        if sha(ROOT / p) != h:
            raise ValueError("Frozen runtime changed " + p)
    models = {}
    for role in ROLES:
        group = (
            "bootstrap"
            if (split == "attack" and role in ("defender", "keeper"))
            or (split == "defense" and role == "manager")
            else "final"
        )
        m = f["models"][group][role]
        p = release / m["path"]
        if (
            sha(p) != m["sha256"]
            or sha(p.with_suffix(".json")) != m["manifest_sha256"]
            or result["models"][role]["sha256"] != m["sha256"]
        ):
            raise ValueError("Wrong or changed model " + role)
        models[role] = m
    if (
        not result["backend"].startswith("webots_")
        or result["manager_target_fallback"] is not False
        or result["scripted_handoff"] is not False
    ):
        raise ValueError("Not native learned-decision inference")
    base, count = SPLITS[split]
    records = result["episodes"]
    if [e["seed"] for e in records] != list(range(base, base + count)):
        raise ValueError("Missing/reordered/changed reserved cases")
    cfg = json.loads((release / models["manager"]["path"]).with_suffix(".json").read_text())[
        "config"
    ]
    counts = Counter()
    families = {}
    directions = Counter()
    options = Counter()
    target_updates = 0
    pass_targets = []
    for e in records:
        goal, stop, collision, outside = validate_episode(e, cfg, models, require_gate=True)
        counts[e["reason"]] += 1
        counts["goals"] += goal
        counts["stops"] += stop
        counts["collisions"] += collision
        counts["clean"] += (goal or stop) and not collision and not outside
        counts["unforced"] += not (goal or stop or collision) or outside
        counts["passes"] += sum(v["type"] == "pass" for v in e["events"])
        counts["shots"] += sum(v["type"] == "shoot" for v in e["events"])
        counts["receptions"] += e["stable_receptions"]
        counts["carry_episodes"] += max(e["controlled_carry"]) >= 0.5
        counts["direct_goals"] += goal and not any(v["type"] == "pass" for v in e["events"])
        family = families.setdefault(e["family"], Counter())
        family["count"] += 1
        family["clean"] += (goal or stop) and not collision and not outside
        family["goals"] += goal
        directions[str(e["attack_sign"])] += 1
        options.update(e["option_counts"])
        target_updates += sum(t["receive_update"] for t in e["decisions"])
        pass_targets.extend(
            t["receive"] for t in e["decisions"] if t["option"] == "pass" and t["receive_update"]
        )
    gate = (
        counts["clean"] / count >= 0.95
        and counts["collisions"] / count <= 0.02
        and counts["unforced"] / count <= 0.03
    )
    if split == "final":
        gate &= (
            counts["goals"] / count >= 0.2
            and all(c["clean"] / c["count"] >= 0.9 for c in families.values())
            and all(c["goals"] / c["count"] >= 0.1 for c in families.values())
            and all(counts[k] >= 20 for k in ("passes", "shots", "receptions", "carry_episodes"))
            and counts["direct_goals"] >= 5
        )
    if split == "defense":
        gate &= counts["stops"] / count >= 0.2
    if split == "demo":
        gate &= counts["clean"] == 9
    for key in (
        "goals",
        "stops",
        "clean",
        "passes",
        "shots",
        "receptions",
        "carry_episodes",
        "direct_goals",
    ):
        if result["summary"][key] != counts[key]:
            raise ValueError("Summary disagrees " + key)
    return {
        "split": split,
        "count": count,
        "passes_gate": bool(gate),
        "counts": dict(counts),
        "families": families,
        "directions": dict(directions),
        "option_control_ticks": dict(options),
        "learned_target_updates": target_updates,
        "pass_target_ranges": [
            [min(t[i] for t in pass_targets), max(t[i] for t in pass_targets)] for i in range(2)
        ]
        if pass_targets
        else [],
        "clean_wilson_95": wilson(counts["clean"], count),
        "goal_wilson_95": wilson(counts["goals"], count),
        "models": models,
        "runtime_verified": True,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=["stage", "freeze", "audit"])
    p.add_argument("--source", type=Path)
    p.add_argument("--release", type=Path, required=True)
    p.add_argument("--result", type=Path)
    p.add_argument("--split", choices=SPLITS)
    p.add_argument("--output", type=Path)
    a = p.parse_args()
    result = (
        stage(a.source, a.release)
        if a.mode == "stage"
        else freeze(a.release)
        if a.mode == "freeze"
        else audit(a.release, a.result, a.split)
    )
    if a.output:
        write(a.output, result)
    print(json.dumps(result, indent=2))
    if a.mode == "audit" and not result["passes_gate"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
