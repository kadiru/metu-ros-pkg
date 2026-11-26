"""Train a short-horizon carrier policy and freeze it before matched Webots qualification."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import gzip
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np

from train_recovery_touch import (
    R,
    PRIOR,
    ROOT,
    WEBOTS,
    read,
    save,
    sha,
    run_matches,
    panel,
    touch_result,
)

WORLD = ROOT / "src/roboturk_ros/webots/worlds/ssl_touch_planner.wbt"
CTRL = WORLD.parent.parent / "controllers/roboturk_touch_planner/roboturk_touch_planner.py"


def population():
    base = read(PRIOR / "policy-2.json") | dict(
        grip=20.0,
        follow=0.57,
        pivot=3.8,
        cruise=0.72,
        own_cost=4.0,
        enemy_cost=5.0,
        own_margin=0.67,
        buffer=1.65,
        gate=0.08,
        align_cost=0.045,
        stick=0.07,
    )
    return [
        base | dict(steps=n, own_margin=m, buffer=b, gate=g, cruise=v, enemy_cost=c)
        for n, m, b, g, v, c in [
            (5, 0.64, 1.65, 0.07, 0.75, 5.0),
            (7, 0.64, 1.9, 0.11, 0.78, 7.0),
            (4, 0.70, 1.48, 0.075, 0.65, 4.0),
            (6, 0.55, 1.75, 0.045, 0.85, 6.0),
        ]
    ]


def isolated(d, i, exe):
    env = dict(
        os.environ,
        ATTACK_RUN=str(d),
        RECOVERY_CASE=str(i),
        PYTHONPATH=str(ROOT / "src/roboturk_core"),
        PATH=str(Path(sys.executable).parent) + os.pathsep + os.environ["PATH"],
    )
    with (d / f"webots-{i:03}.log").open("w") as f:
        subprocess.run(
            [
                str(exe),
                "--batch",
                "--mode=fast",
                "--no-rendering",
                "--minimize",
                "--stdout",
                "--stderr",
                str(WORLD),
            ],
            env=env,
            stdout=f,
            stderr=subprocess.STDOUT,
            check=True,
        )
    assert (d / f"game-{i:03}.json").exists()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "mode",
        choices=["train", "dev", "known", "freeze", "qualify", "control", "historical", "demo"],
    )
    p.add_argument("folder", type=Path)
    p.add_argument("--models", type=Path)
    p.add_argument("--version", type=int, default=2)
    p.add_argument("--offset", type=int, default=0)
    p.add_argument("--cases", type=int, default=24)
    p.add_argument("--cross", action="store_true")
    p.add_argument("--workers", type=int, default=2, choices=(1, 2))
    p.add_argument("--webots", type=Path, default=WEBOTS)
    a = p.parse_args()
    plan = read(R / "protocol.json")
    prior = read(PRIOR / "policy-2.json")
    if a.version < 2 or not 0 <= a.offset < 90000 or not 1 <= a.cases <= 500:
        p.error("Invalid range")
    checkpoint = R / f"planned-policy-{a.version}.json"
    lock = R / f"planned-freeze-{a.version}.json"
    for f, digest in read(R / "freeze-1.json").items():
        if sha(ROOT / f) != digest:
            raise ValueError(f"First-touch freeze mismatch in {f}")
    for f, digest in read(PRIOR / "freeze-2.json").items():
        if sha(ROOT / f) != digest:
            raise ValueError(f"Pressure freeze mismatch in {f}")
    if a.mode == "freeze":
        if checkpoint.exists() or lock.exists():
            p.error("Previously frozen version")
        save(checkpoint, read(a.folder / "policy.json"))
        files = [
            WORLD,
            CTRL,
            ROOT / "src/roboturk_core/roboturk/carrier_motor.py",
            ROOT / "src/roboturk_core/roboturk/touch_planner.py",
            Path(__file__).resolve(),
            R / "freeze-1.json",
            checkpoint,
            R / "protocol.json",
        ]
        save(lock, {str(f.relative_to(ROOT)): sha(f) for f in files})
        return
    frozen = a.mode in ("qualify", "control", "historical", "demo")
    if frozen:
        if a.models or a.offset or a.cross:
            p.error("Registered panels require the frozen checkpoint and starts")
        for f, digest in read(lock).items():
            if sha(ROOT / f) != digest:
                raise ValueError(f"Freeze mismatch in {f}")
    models = (
        [prior if a.mode == "control" else read(checkpoint)]
        if frozen
        else read(a.models)
        if a.models
        else population()
    )
    if isinstance(models, dict):
        models = [models]
    n = len(models)
    if a.mode not in ("train", "known") and n != 1:
        p.error("One team required")
    first = (
        plan[
            {"train": "train", "dev": "development", "demo": "demo"}.get(a.mode, "qualification")
            + "_first"
        ]
        + (a.version - 1) * plan["version_stride"]
        + a.offset
    )
    count = 40 if a.mode in ("qualify", "control") else 3 if a.mode == "demo" else a.cases
    schedule = [
        [first + i, j, (i + j) % n if a.cross else j] for i in range(count) for j in range(n)
    ]
    if a.mode in ("known", "historical"):
        schedule = []
        for loss in read(R / "original-losses.json"):
            for j in range(n):
                schedule.append(
                    [loss["seed"], j, n] if loss["team"] == "blue" else [loss["seed"], n, j]
                )
        models = models + [prior]
    d = a.folder.resolve()
    d.mkdir(parents=True, exist_ok=False)
    cfg = dict(
        root=str(ROOT),
        world=str(WORLD.relative_to(ROOT)),
        models=models,
        cases=len(schedule),
        schedule=schedule,
        protocol=plan,
        seconds=18,
        study=a.mode == "train",
        press=True,
        record=a.mode == "demo",
        webots_sha256=sha(a.webots),
    )
    save(d / "launch.json", cfg)
    if a.mode in ("qualify", "control", "known", "historical"):
        with ThreadPoolExecutor(max_workers=a.workers) as pool:
            list(pool.map(lambda i: isolated(d, i, a.webots), range(len(schedule))))
    else:
        run_matches(d, cfg, a.webots, WORLD, False)
    games = [read(d / f"game-{i:03}.json") for i in range(len(schedule))]
    rows = [g["opening"] for g in games]
    if a.mode == "train":
        stats = []
        for j in range(n):
            owned = [
                r
                for g, r in zip(games, rows, strict=True)
                if (t := r["chain"].get("team")) and g[t + "_model"] == j
            ]
            stats.append(panel(owned))
        best = max(
            range(n),
            key=lambda j: (stats[j]["goals"] - 2 * stats[j]["prepass_losses"], stats[j]["goals"]),
        )
        save(d / "policy.json", models[best])
        save(d / "returns.json", stats)
        data = dict(
            config={k: v for k, v in cfg.items() if k != "root"},
            returns=stats,
            selected=best,
            games=games,
        )
        (d / "training.json.gz").write_bytes(
            gzip.compress(json.dumps(data, separators=(",", ":")).encode(), mtime=0)
        )
        print(stats)
        return
    for i, (r, g, scheduled) in enumerate(zip(rows, games, schedule, strict=True)):
        s = np.load(d / f"trace-{i:03}.npz")["states"]
        assert len(s) == 1800 and g["seed"] == scheduled[0] and touch_result(s, g, plan) == r
    result = panel(rows) | dict(
        contacts=sum(g["stats"]["contacts"] for g in games),
        exits=sum(g["stats"]["exits"] for g in games),
    )
    if a.mode == "qualify":
        result["passed"] = (
            len(rows) == 40
            and result["goals"] >= plan["goal_target"]
            and result["prepass_losses"] <= plan["loss_limit"]
        )
    save(d / "audit.json", result | dict(rows=rows, passing_goals=result["goals"]))
    print(result)


if __name__ == "__main__":
    main()
