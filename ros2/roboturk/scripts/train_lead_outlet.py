"""Train leading-pass teams and evaluate matched, independently launched Webots starts."""

import argparse
from collections import Counter
from pathlib import Path
import gzip
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from run_recovery_attack import ROOT, read, save
from train_recovery_transition import run_matches
from train_blocked_outlet import sha
from roboturk.lead_audit import lead_result

R = ROOT / "policies/lead_outlet_v1"
PRIOR = ROOT / "policies/blocked_outlet_v1"
WORLD = ROOT / "src/roboturk_ros/webots/worlds/ssl_lead.wbt"


def population():
    base = read(PRIOR / "policy-1.json") | dict(
        run_speed=0.59,
        roll=1.6,
        horizon=0.92,
        min_lead=0.30,
        forecast=0.65,
        lane_weight=2.8,
        progress=0.30,
        space_weight=0.42,
        cut=0.18,
        aim_time=0.16,
        angle=0.042,
        release_gap=0.23,
        ready=0.18,
    )
    return [
        base,
        base | dict(run_speed=0.79, horizon=1.0, progress=0.5, cut=0.10, release_gap=0.26),
        base
        | dict(
            run_speed=0.53, forecast=0.9, cut=0.54, progress=0.12, release_gap=0.18, aim_time=0.10
        ),
        base | dict(run_speed=0.71, cut=0.62, progress=0.26, horizon=0.80, release_gap=0.20),
    ]


def evaluate_isolated(d, cfg, exe, workers):
    # Each 40-game panel has 40 process launches and independent ODE state.
    def one(i):
        env = dict(
            os.environ,
            ATTACK_RUN=str(d),
            RECOVERY_CASE=str(i),
            PYTHONPATH=str(ROOT / "src/roboturk_core"),
            PATH=str(Path(sys.executable).parent) + os.pathsep + os.environ["PATH"],
        )
        with (d / f"webots-{i:03}.log").open("w") as log:
            subprocess.run(
                [
                    str(exe),
                    "--batch",
                    "--stdout",
                    "--stderr",
                    "--mode=fast",
                    "--no-rendering",
                    "--minimize",
                    str(WORLD),
                ],
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                check=True,
            )
        if not (d / f"game-{i:03}.json").exists():
            raise RuntimeError(f"Missing native opening {i} in {d}")

    with ThreadPoolExecutor(max_workers=workers) as pool:
        list(pool.map(one, range(cfg["cases"])))


def summary(rows):
    return dict(
        games=len(rows),
        goals=sum(r["pass_goal"] for r in rows),
        lead_goals=sum(r["qualified"] for r in rows),
        interceptions=sum(r["intercepted"] for r in rows),
        outcomes=dict(Counter(r["outcome"] for r in rows)),
    )


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "mode", choices=["train", "dev", "freeze", "qualify", "baseline", "historical", "demo"]
    )
    p.add_argument("folder", type=Path)
    p.add_argument("--models", type=Path)
    p.add_argument("--offset", type=int, default=0)
    p.add_argument("--cases", type=int, default=24)
    p.add_argument("--version", type=int, default=1)
    p.add_argument("--mirror", action="store_true")
    p.add_argument("--workers", type=int, choices=(1, 2), default=1)
    p.add_argument(
        "--webots",
        type=Path,
        default=ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots",
    )
    a = p.parse_args()
    plan = read(R / "protocol.json")
    policy = R / f"policy-{a.version}.json"
    freeze = R / f"freeze-{a.version}.json"
    for name, digest in read(PRIOR / "freeze-1.json").items():
        if sha(ROOT / name) != digest:
            raise ValueError(f"Prior freeze mismatch in {name}")
    if a.mode == "freeze":
        if policy.exists() or freeze.exists():
            p.error("Checkpoint version already frozen")
        save(policy, read(a.folder / "policy.json"))
        files = [
            WORLD,
            ROOT / "src/roboturk_ros/webots/controllers/roboturk_lead/roboturk_lead.py",
            Path(__file__).resolve(),
            ROOT / "src/roboturk_core/roboturk/lead_outlet.py",
            ROOT / "src/roboturk_core/roboturk/lead_audit.py",
            ROOT / "src/roboturk_ros/webots/protos/SslBall.proto",
            ROOT / "src/roboturk_ros/webots/protos/SslDivisionBField.proto",
            ROOT / "src/roboturk_ros/webots/protos/SslRobot.proto",
            ROOT / "src/roboturk_core/roboturk/recovery_attack.py",
            ROOT / "scripts/compare_lead_openings.py",
            ROOT / "scripts/evolve_lead_outlet.py",
            R / "previous-interceptions.json",
            PRIOR / "freeze-1.json",
            R / "protocol.json",
            policy,
        ]
        save(freeze, {str(f.relative_to(ROOT)): sha(f) for f in files})
        return
    frozen = a.mode in ("qualify", "baseline", "historical", "demo")
    if frozen:
        if a.models or a.offset:
            p.error("Reserved panels require frozen models and registered starts")
        for name, digest in read(freeze).items():
            if sha(ROOT / name) != digest:
                raise ValueError(f"Freeze mismatch in {name}")
    if a.version < 1 or not 0 <= a.offset < 90000 or a.cases < 1:
        p.error("Invalid range")
    models = (
        population()
        if a.mode == "train"
        else [read(PRIOR / "policy-1.json")]
        if a.mode == "baseline"
        else [read(policy)]
        if not a.models
        else []
    )
    if a.models:
        m = read(a.models)
        models = m if isinstance(m, list) else [m]
    if a.mode != "train" and len(models) != 1:
        p.error("A single team is required")
    first = (
        plan[
            {"train": "train", "dev": "development", "demo": "demo"}.get(a.mode, "qualification")
            + "_first"
        ]
        + a.offset
        + (a.version - 1) * plan["version_stride"]
    )
    count = (
        plan["cases"] if a.mode in ("qualify", "baseline") else 3 if a.mode == "demo" else a.cases
    )
    n = len(models)
    schedule = [
        [first + i, j, j if a.mirror else (i + j) % n] for i in range(count) for j in range(n)
    ]
    if a.mode == "historical":
        schedule = [[v["seed"], 0, 0] for v in read(R / "previous-interceptions.json")]
    d = a.folder.resolve()
    d.mkdir(parents=True, exist_ok=False)
    cfg = dict(
        root=str(ROOT),
        protocol=plan,
        models=models,
        schedule=schedule,
        cases=len(schedule),
        record=a.mode == "demo",
        seconds=plan["seconds"],
        study=a.mode == "train",
        webots_sha256=sha(a.webots),
    )
    save(d / "launch.json", cfg)
    if a.mode in ("qualify", "baseline", "historical"):
        evaluate_isolated(d, cfg, a.webots, a.workers)
    else:
        run_matches(d, cfg, a.webots, WORLD, False)
    games = [read(f) for f in sorted(d.glob("game-*.json"))]
    if a.mode == "train":
        stats = []
        for j in range(n):
            rows = [
                g["opening"]
                for g in games
                if (t := g["opening"].get("chain", {}).get("team")) and g[t + "_model"] == j
            ]
            stats.append(summary(rows) | dict(candidate=j))
        best = max(
            stats, key=lambda x: (x["lead_goals"] - 0.7 * x["interceptions"], x["lead_goals"])
        )
        save(d / "returns.json", stats)
        save(d / "policy.json", models[best["candidate"]])
        archive = dict(
            config={k: v for k, v in cfg.items() if k != "root"}, returns=stats, games=games
        )
        (d / "training.json.gz").write_bytes(
            gzip.compress(json.dumps(archive, separators=(",", ":")).encode(), mtime=0)
        )
        print(stats)
    else:
        for i, g in enumerate(games):
            s = np.load(d / f"trace-{i:03}.npz")["states"]
            assert len(s) == cfg["seconds"] * 100 and g["seed"] == schedule[i][0]
            assert lead_result(s, g, plan) == g["opening"]
        rows = [g["opening"] for g in games]
        result = summary(rows) | dict(
            contacts=sum(g["stats"]["contacts"] for g in games),
            exits=sum(g["stats"]["exits"] for g in games),
        )
        if a.mode == "qualify":
            result["passed"] = (
                len(rows) == 40 and result["lead_goals"] >= 24 and result["interceptions"] <= 8
            )
        save(d / "audit.json", result | dict(rows=rows, passing_goals=result["lead_goals"]))
        print(result)


if __name__ == "__main__":
    main()
