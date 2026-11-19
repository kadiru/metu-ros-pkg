#!/usr/bin/env python3
"""Freeze all four actors and audit complete native episodes without filtering."""

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo
from audit_micro_release import audit

ROOT = Path(__file__).resolve().parents[1]
FIELDS = {
    "transfer": "checkpoint_sha256",
    "finish": "shooting_checkpoint_sha256",
    "defender": "defender_checkpoint_sha256",
    "keeper": "keeper_checkpoint_sha256",
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def freeze(args):
    if args.output.exists():
        raise ValueError("Refusing to overwrite an existing freeze")
    models = {}
    for role in FIELDS:
        path = getattr(args, role).resolve()
        models[role] = {
            "path": str(path.relative_to(ROOT)),
            "sha256": digest(path),
            "manifest_sha256": digest(path.with_suffix(".json")),
        }
    paths = (
        sorted(ROOT.glob("src/**/*.py"))
        + sorted(ROOT.glob("src/**/*.proto"))
        + sorted(ROOT.glob("src/**/*.wbt"))
        + [
            ROOT / "docs/runtime-manifest.json",
            ROOT / "requirements-macos-validation.txt",
            ROOT / "src/roboturk_ros/webots/controllers/roboturk_supervisor/runtime.ini",
            ROOT / "scripts/run_webots.py",
            Path(__file__).resolve(),
            ROOT / "scripts/audit_micro_release.py",
        ]
    )
    data = {
        "created_at": datetime.now(ZoneInfo("America/Los_Angeles")).isoformat(),
        "models": models,
        "runtime_sha256": {str(p.relative_to(ROOT)): digest(p) for p in paths},
        "qualification_seeds": [1650000, 1650199],
        "demo_seeds": [1660000, 1660005],
        "gate": "at least95% complete goals, zero pre-release possession losses and collisions",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2) + "\n")
    return {"freeze": str(args.output), "models": models}


def audit_release(args):
    frozen = json.loads(args.freeze.read_text())
    result = json.loads(args.result.read_text())
    if args.result.stat().st_mtime < datetime.fromisoformat(frozen["created_at"]).timestamp():
        raise ValueError("Result predates checkpoint freeze")
    for path, expected in frozen["runtime_sha256"].items():
        if digest(ROOT / path) != expected:
            raise ValueError("Frozen runtime changed: " + path)
    for role, model in frozen["models"].items():
        path = ROOT / model["path"]
        if (
            digest(path) != model["sha256"]
            or digest(path.with_suffix(".json")) != model["manifest_sha256"]
        ):
            raise ValueError("Frozen model or manifest changed: " + role)
        if result[FIELDS[role]] != model["sha256"]:
            raise ValueError("Evaluation did not use the frozen " + role)
    if result.get("opponent_policy_fallback") is not False or not result["backend"].startswith(
        "webots_"
    ):
        raise ValueError("Expected native Webots and explicit absence of opponent fallback")
    report = audit(result, args.seed, args.count, "play")
    calls = {"defender": 0, "keeper": 0}
    for record in result["episodes"]:
        transfer, shot = record["transfer"], record["shooting"]
        total = transfer["steps"] + (shot["steps"] if shot else 0)
        last = shot or transfer
        if last["opponent"] != "learned_lane_v1" or last["keeper"] != "learned_keeper_v1":
            raise ValueError("Analytic opponent appeared in qualification")
        for role in calls:
            if last[role + "_checkpoint_sha256"] != frozen["models"][role]["sha256"]:
                raise ValueError("Per-episode opponent hash mismatch")
            if last[role + "_policy_calls"] != total or last[role + "_physics_checks"] != total * 4:
                raise ValueError("Opponent was not active on every control/physics frame")
            calls[role] += last[role + "_policy_calls"]
    report.update(
        policy_calls=calls,
        models=frozen["models"],
        runtime_verified=True,
        wilson_95=result["wilson_95"],
        mean_success_seconds=result["mean_success_seconds"],
    )
    return report


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="mode", required=True)
    f = sub.add_parser("freeze")
    for role in FIELDS:
        f.add_argument("--" + role, type=Path, required=True)
    f.add_argument("--output", type=Path, required=True)
    a = sub.add_parser("audit")
    a.add_argument("--freeze", type=Path, required=True)
    a.add_argument("--result", type=Path, required=True)
    a.add_argument("--seed", type=int, required=True)
    a.add_argument("--count", type=int, default=200)
    a.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = freeze(args) if args.mode == "freeze" else audit_release(args)
    print(json.dumps(report, indent=2))
    if args.mode == "audit" and args.output:
        args.output.write_text(json.dumps(report, indent=2) + "\n")
    if not report.get("passes_gate", True):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
