"""Run native candidate leagues, freeze a team, and audit unseen blocked-lane openings."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np

from train_recovery_transition import run_matches
from run_recovery_attack import ROOT, read, save

R = ROOT / "policies/blocked_outlet_v1"
WORLD = ROOT / "src/roboturk_ros/webots/worlds/ssl_blocked.wbt"
SOURCES = [
    Path(__file__).resolve(),
    WORLD,
    R / "protocol.json",
    ROOT / "src/roboturk_ros/webots/controllers/roboturk_blocked/roboturk_blocked.py",
    ROOT / "src/roboturk_core/roboturk/blocked_outlet.py",
    ROOT / "src/roboturk_core/roboturk/blocked_audit.py",
]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def population():
    prev = read(ROOT / "policies/recovery_attack_v3/candidate-1.json")
    base = {k: prev[k] for k in ("tracking", "centering", "turn", "escape", "speed")}
    base.update(
        receiver=[2.2, 0.52, 0.34, 0.16, -0.20],
        bias=0.75,
        block=2.6,
        lane=0.85,
        support_x=0.95,
        support_y=1.28,
        cover=1.04,
        finish=1.2,
    )
    return [
        base,
        base | dict(bias=0.64, turn=4.6, tracking=0.54, cover=0.85),
        base
        | dict(receiver=[2.8, 0.85, 0.18, 0.4, -0.4], escape=0.62, support_x=1.25, finish=1.65),
        base | dict(bias=1.02, block=3.0, turn=3.1, cover=1.23, support_y=1.55),
    ]


def summarize(games):
    rows = [g["opening"] for g in games]
    return dict(
        games=len(rows),
        passing_goals=sum(r["qualified"] for r in rows),
        outcomes=dict(Counter(r["outcome"] for r in rows)),
        rows=rows,
    )


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("mode", choices=["train", "dev", "freeze", "qualify", "demo"])
    p.add_argument("folder", type=Path)
    p.add_argument("--cases", type=int, default=24, help="training layouts or development games")
    p.add_argument(
        "--models", type=Path, help="training population or a single development checkpoint"
    )
    p.add_argument("--offset", type=int, default=0, help="training/development seed offset")
    p.add_argument("--version", type=int, default=1)
    p.add_argument(
        "--webots",
        type=Path,
        default=ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots",
    )
    a = p.parse_args()
    plan = read(R / "protocol.json")
    policy = R / f"policy-{a.version}.json"
    freeze = R / f"freeze-{a.version}.json"
    if not 0 <= a.offset < 90000 or a.version < 1 or a.cases < 1:
        p.error("Invalid seed allocation or case count")
    if a.mode == "freeze":
        if policy.exists() or freeze.exists():
            p.error("Checkpoint version already exists")
        save(policy, read(a.folder / "policy.json"))
        # Include inherited sources, simulation constants and prior learned components.
        inherited = read(ROOT / "policies/recovery_attack_v3/freeze-1.json")
        old = [ROOT / k for k in inherited]
        old.extend(
            [
                ROOT / "src/roboturk_core/roboturk" / f
                for f in (
                    "contest_metrics.py",
                    "recovery_conversion.py",
                    "three_match.py",
                    "contest_outlet.py",
                    "jaw_recovery.py",
                    "continuous_backend.py",
                    "webots_backend.py",
                    "model.py",
                    "three_recipient.py",
                )
            ]
        )
        old.extend(
            [
                ROOT / x
                for x in (
                    "policies/contest_outlet_v2/policy.json",
                    "policies/recipient_3v3_v1/selectors.json",
                    "policies/strategy_self_play_v1/preparation/pass-motor-v5/policy.pt",
                )
            ]
        )
        save(freeze, {str(k.relative_to(ROOT)): sha(k) for k in [*old, *SOURCES, policy]})
        return
    if a.mode in ("qualify", "demo"):
        if a.models or a.offset:
            p.error("Qualification and demo use only the frozen checkpoint and registered seeds")
        for k, h in read(freeze).items():
            if sha(ROOT / k) != h:
                raise ValueError(f"Freeze mismatch in {k}")
    models = population() if a.mode == "train" else [read(policy)] if not a.models else []
    if a.models:
        value = read(a.models)
        models = value if isinstance(value, list) else [value]
    if a.mode != "train" and len(models) != 1:
        p.error("Expected exactly one checkpoint")
    key = {"train": "train", "dev": "development", "qualify": "qualification", "demo": "demo"}[
        a.mode
    ]
    first = plan[key + "_first"] + a.offset + (a.version - 1) * plan["version_stride"]
    n = len(models)
    if a.mode == "train":
        schedule = [[first + i, j, (j + i) % n] for i in range(a.cases) for j in range(n)]
    else:
        count = plan["cases"] if a.mode == "qualify" else 3 if a.mode == "demo" else a.cases
        schedule = [[first + i, 0, 0] for i in range(count)]
    d = a.folder.resolve()
    d.mkdir(parents=True, exist_ok=False)
    cfg = dict(
        root=str(ROOT),
        cases=len(schedule),
        schedule=schedule,
        models=models,
        protocol=plan,
        seconds=plan["seconds"],
        record=a.mode == "demo",
        study=a.mode == "train",
        webots_sha256=sha(a.webots),
    )
    save(d / "launch.json", cfg)
    run_matches(d, cfg, a.webots, WORLD, a.mode == "qualify")
    games = [read(f) for f in sorted(d.glob("game-*.json"))]
    if a.mode == "train":
        table = []
        for j in range(n):
            group = [
                g
                for g in games
                if "team" in g["opening"].get("chain", {})
                and g[g["opening"]["chain"]["team"] + "_model"] == j
            ]
            result = summarize(group)
            table.append(
                dict(
                    candidate=j,
                    opportunities=len(group),
                    goals=result["passing_goals"],
                    receives=sum(r.get("pass_result") == "complete" for r in result["rows"]),
                    outcomes=result["outcomes"],
                )
            )
        best = max(table, key=lambda x: (x["goals"], x["receives"]))
        save(d / "returns.json", table)
        save(d / "policy.json", models[best["candidate"]])
        print(table)
    else:
        from roboturk.blocked_audit import opening_result

        for i, g in enumerate(games):
            states = np.load(d / f"trace-{i:03}.npz")["states"]
            assert len(states) == plan["seconds"] * 100
            assert opening_result(states, g, plan) == g["opening"]
        report = summarize(games)
        report["contacts"] = sum(g["stats"]["contacts"] for g in games)
        report["exits"] = sum(g["stats"]["exits"] for g in games)
        if a.mode == "qualify":
            report["passed"] = len(games) == 40 and report["passing_goals"] >= plan["goal_target"]
        save(d / "audit.json", report)
        print({k: v for k, v in report.items() if k != "rows"})


if __name__ == "__main__":
    main()
