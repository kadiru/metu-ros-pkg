"""Native self-play search and matched qualification for the carrier's first touch."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import gzip
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
from roboturk.touch_audit import touch_result
from train_pressure_receive import ROOT, WEBOTS, read, save, sha, run_matches, totals

R = ROOT / "policies/recovery_touch_v1"
PRIOR = ROOT / "policies/pressure_receive_v1"
WORLD = ROOT / "src/roboturk_ros/webots/worlds/ssl_touch.wbt"
CTRL = WORLD.parent.parent / "controllers/roboturk_touch/roboturk_touch.py"


def population():
    base = read(PRIOR / "policy-2.json") | dict(
        look=0.07, rush_weight=1.55, shield_bias=0.30, side_bias=0.55, side_cost=0.12, window=105
    )
    return [
        base | dict(release_value=z, cruise=v, follow=a, grip=g, pivot=r, brake=b)
        for z, v, a, g, r, b in [
            (1.35, 0.58, 0.52, 19, 4.5, 0.80),
            (1.0, 0.48, 0.70, 21, 3.5, 0.72),
            (1.65, 0.65, 0.46, 19, 4.1, 0.78),
            (1.2, 0.68, 0.68, 23, 4.2, 0.80),
        ]
    ]


def panel(rows):
    return totals(rows) | dict(
        prepass_losses=sum(r["prepass_loss"] for r in rows),
        confirmed=sum("winner" in r.get("chain", {}) for r in rows),
    )


def single(d, i, exe):
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
                "--mode=fast",
                "--no-rendering",
                "--minimize",
                "--stdout",
                "--stderr",
                str(WORLD),
            ],
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            check=True,
        )
    if not (d / f"game-{i:03}.json").exists():
        raise RuntimeError(f"Missing native game {i} in {d}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "mode",
        choices=[
            "train",
            "dev",
            "known",
            "old5",
            "freeze",
            "qualify",
            "control",
            "demo",
            "historical",
        ],
    )
    parser.add_argument("folder", type=Path)
    parser.add_argument("--models", type=Path)
    parser.add_argument("--cases", type=int, default=16)
    parser.add_argument("--offset", type=int, default=0)
    parser.add_argument("--version", type=int, default=1)
    parser.add_argument("--mirror", action="store_true")
    parser.add_argument("--workers", type=int, choices=(1, 2), default=2)
    parser.add_argument("--webots", type=Path, default=WEBOTS)
    a = parser.parse_args()
    plan = read(R / "protocol.json")
    prior = read(PRIOR / "policy-2.json")
    for name, digest in read(PRIOR / "freeze-2.json").items():
        if sha(ROOT / name) != digest:
            raise ValueError(f"Prior freeze mismatch {name}")
    version = a.version
    checkpoint = R / f"policy-{version}.json"
    lock = R / f"freeze-{version}.json"
    if a.mode == "freeze":
        if checkpoint.exists() or lock.exists():
            parser.error("Already frozen")
        save(checkpoint, read(a.folder / "policy.json"))
        sources = [
            WORLD,
            CTRL,
            ROOT / "src/roboturk_core/roboturk/recovery_touch.py",
            ROOT / "src/roboturk_core/roboturk/touch_audit.py",
            Path(__file__).resolve(),
            R / "protocol.json",
            checkpoint,
            PRIOR / "freeze-2.json",
            PRIOR / "policy-2.json",
        ]
        save(lock, {str(p.relative_to(ROOT)): sha(p) for p in sources})
        return
    frozen = a.mode in ("qualify", "control", "demo", "historical")
    if frozen:
        if a.models or a.offset:
            parser.error("Frozen runs require the registered seeds and policies")
        for name, digest in read(lock).items():
            if sha(ROOT / name) != digest:
                raise ValueError(f"Freeze mismatch in {name}")
    models = (
        [prior if a.mode == "control" else read(checkpoint)]
        if frozen
        else [prior]
        if a.mode == "old5"
        else read(a.models)
        if a.models
        else population()
    )
    if isinstance(models, dict):
        models = [models]
    if a.mode != "train" and a.mode != "known" and len(models) != 1:
        parser.error("One team required")
    if not (1 <= a.cases <= 500 and 0 <= a.offset < 90000 and version >= 1):
        parser.error("Invalid range")
    n = len(models)
    count = 40 if a.mode in ("qualify", "control") else 3 if a.mode == "demo" else a.cases
    first = (
        plan[
            {"train": "train", "dev": "development", "demo": "demo"}.get(a.mode, "qualification")
            + "_first"
        ]
        + a.offset
        + (version - 1) * plan["version_stride"]
    )
    schedule = [
        [first + i, j, j if a.mirror else (i + j) % n] for i in range(count) for j in range(n)
    ]
    known = a.mode in ("known", "old5", "historical")
    if known:
        schedule = []
        old = PRIOR / "qualification-2"
        for seed in plan["historical"]:
            i = seed - read(old / "launch.json")["schedule"][0][0]
            blue = read(old / f"game-{i:03}.json")["opening"]["chain"]["team"] == "blue"
            for j in range(n):
                schedule.append([seed, j, n] if blue else [seed, n, j])
        models = models + [prior]
    d = a.folder.resolve()
    d.mkdir(parents=True, exist_ok=False)
    cfg = dict(
        root=str(ROOT),
        protocol=plan,
        models=models,
        schedule=schedule,
        cases=len(schedule),
        seconds=18,
        study=a.mode == "train",
        press=True,
        record=a.mode == "demo",
        webots_sha256=sha(a.webots),
    )
    save(d / "launch.json", cfg)
    if a.mode in ("qualify", "control") or known:
        with ThreadPoolExecutor(max_workers=a.workers) as pool:
            list(pool.map(lambda i: single(d, i, a.webots), range(len(schedule))))
    else:
        run_matches(d, cfg, a.webots, WORLD, False)
    games = [read(d / f"game-{i:03}.json") for i in range(len(schedule))]
    rows = [g["opening"] for g in games]
    if a.mode == "train":
        stats = []
        for j in range(n):
            owned = [
                r
                for r, g in zip(rows, games, strict=True)
                if (t := r.get("chain", {}).get("team")) and g[t + "_model"] == j
            ]
            stats.append(panel(owned))
        best = max(
            range(n),
            key=lambda j: (stats[j]["goals"] - 0.9 * stats[j]["prepass_losses"], stats[j]["goals"]),
        )
        save(d / "policy.json", models[best])
        save(d / "returns.json", stats)
        archive = dict(
            config={k: v for k, v in cfg.items() if k != "root"},
            games=games,
            returns=stats,
            selected=best,
        )
        (d / "training.json.gz").write_bytes(
            gzip.compress(json.dumps(archive, separators=(",", ":")).encode(), mtime=0)
        )
        print(stats)
    else:
        for i, (g, expected) in enumerate(zip(games, schedule, strict=True)):
            states = np.load(d / f"trace-{i:03}.npz")["states"]
            assert (
                len(states) == 1800
                and g["seed"] == expected[0]
                and touch_result(states, g, plan) == rows[i]
            )
            if a.mode == "old5":
                j = g["seed"] - read(old / "launch.json")["schedule"][0][0]
                assert np.array_equal(states, np.load(old / f"trace-{j:03}.npz")["states"])
                original = read(old / f"game-{j:03}.json")
                assert all(g[k] == original[k] for k in ("events", "passes", "options", "plans"))
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
