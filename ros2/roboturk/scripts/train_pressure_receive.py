"""Search receiver shields in 3v3 Webots self-play and test a frozen pair of teams."""

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import gzip
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
from roboturk.pressure_audit import pressure_result
from run_recovery_attack import ROOT, read, save
from train_blocked_outlet import sha
from train_recovery_transition import run_matches

R = ROOT / "policies/pressure_receive_v1"
PRIOR = ROOT / "policies/lead_outlet_v1"
WORLD = ROOT / "src/roboturk_ros/webots/worlds/ssl_pressure.wbt"
CONTROLLER = WORLD.parent.parent / "controllers/roboturk_pressure/roboturk_pressure.py"
WEBOTS = ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots"


def population(base, generation):
    if not generation:
        return [
            base | dict(shield_speed=v, shield_turn=r, hold=h, dwell=n, press_speed=1.05)
            for v, r, h, n in [
                (0.45, 5.7, 13, 7),
                (0.80, 6.4, 14, 8),
                (1.1, 6.5, 17, 9),
                (0.64, 4.5, 16, 8),
            ]
        ]
    rng = np.random.default_rng(7821 + generation)
    models = [base]
    for _ in range(3):
        child = dict(base)
        for key, lo, hi in [
            ("shield_speed", 0.34, 1.15),
            ("shield_turn", 4, 8),
            ("reach", 0.55, 1.05),
            ("min_lead", 0.22, 0.36),
            ("run_speed", 0.49, 0.74),
            ("advance", 0.82, 1.32),
            ("slide", 0.12, 0.52),
        ]:
            child[key] = round(
                float(np.clip(base[key] + rng.normal(0, (hi - lo) * 0.20), lo, hi)), 5
            )
        child["hold"] = int(np.clip(base["hold"] + rng.integers(-4, 5), 9, 22))
        models.append(child)
    return models


def totals(rows):
    return dict(
        games=len(rows),
        goals=sum(r["qualified"] for r in rows),
        leads=sum(r["lead_goal"] for r in rows),
        interceptions=sum(r["intercepted"] for r in rows),
        under_pressure=sum(r.get("pressure", False) for r in rows),
        shields=sum(r.get("pressure", False) and r.get("shield", False) for r in rows),
        outcomes=dict(Counter(r["outcome"] for r in rows)),
    )


def isolated(folder, index, exe):
    env = dict(
        os.environ,
        ATTACK_RUN=str(folder),
        RECOVERY_CASE=str(index),
        PYTHONPATH=str(ROOT / "src/roboturk_core"),
        PATH=str(Path(sys.executable).parent) + os.pathsep + os.environ["PATH"],
    )
    with (folder / f"webots-{index:03}.log").open("w") as log:
        subprocess.run(
            [
                str(exe),
                "--batch",
                "--mode=fast",
                "--stdout",
                "--stderr",
                "--no-rendering",
                "--minimize",
                str(WORLD),
            ],
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            check=True,
        )
    if not (folder / f"game-{index:03}.json").exists():
        raise RuntimeError(f"Missing game {index} in {folder}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "mode", choices=["train", "dev", "freeze", "qualify", "control", "historical", "demo"]
    )
    parser.add_argument("folder", type=Path)
    parser.add_argument("--base", type=Path)
    parser.add_argument("--league", type=Path, help="Explicit candidate list for mirror selection")
    parser.add_argument("--generation", type=int, default=0)
    parser.add_argument("--offset", type=int, default=0)
    parser.add_argument("--cases", type=int, default=16)
    parser.add_argument("--version", type=int, default=1)
    parser.add_argument("--webots", type=Path, default=WEBOTS)
    a = parser.parse_args()
    plan = read(R / "protocol.json")
    policy = R / f"policy-{a.version}.json"
    lock = R / f"freeze-{a.version}.json"
    prior = PRIOR / "policy-1.json"
    source = [
        WORLD,
        CONTROLLER,
        ROOT / "src/roboturk_core/roboturk/pressure_audit.py",
        ROOT / "src/roboturk_core/roboturk/pressure_receive.py",
        Path(__file__).resolve(),
        R / "protocol.json",
        policy,
        PRIOR / "freeze-1.json",
    ]
    for name, digest in read(PRIOR / "freeze-1.json").items():
        if sha(ROOT / name) != digest:
            raise ValueError(f"Previous freeze mismatch in {name}")
    for name in (
        "three_match",
        "three_recipient",
        "webots_backend",
        "continuous_backend",
        "model",
        "blocked_audit",
        "blocked_outlet",
        "contest_metrics",
        "contest_outlet",
        "jaw_recovery",
        "transition_attack",
        "recovery_conversion",
    ):
        source.append(ROOT / f"src/roboturk_core/roboturk/{name}.py")
    source += [
        ROOT / "policies/strategy_self_play_v1/preparation/pass-motor-v5/policy.pt",
        ROOT / "policies/contest_outlet_v2/policy.json",
        ROOT / "policies/recipient_3v3_v1/selectors.json",
        prior,
    ]
    if a.league and a.mode != "train":
        parser.error("A candidate league requires training mode")
    if a.mode == "freeze":
        if lock.exists() or policy.exists():
            parser.error("Version already frozen")
        save(policy, read(a.folder / "policy.json"))
        save(lock, {str(p.relative_to(ROOT)): sha(p) for p in source})
        return
    frozen = a.mode not in ("train", "dev")
    if frozen:
        if a.base or a.offset or a.generation:
            parser.error("Reserved panels require immutable policies and starts")
        for name, digest in read(lock).items():
            if sha(ROOT / name) != digest:
                raise ValueError(f"Freeze mismatch in {name}")
    if a.version < 1 or not 0 <= a.offset < 90000 or not 1 <= a.cases <= 500:
        parser.error("Invalid seed range or count")
    directory = a.folder.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    models = (
        [read(prior if a.mode == "control" else policy)]
        if frozen
        else population(read(a.base or prior), a.generation)
        if a.mode == "train"
        else [read(a.base or prior)]
    )
    if a.league:
        models = read(a.league)
    count = 40 if a.mode in ("control", "qualify") else 3 if a.mode == "demo" else a.cases
    first = plan[
        {"train": "train", "dev": "development", "demo": "demo"}.get(a.mode, "qualification")
        + "_first"
    ]
    first += a.offset + (a.version - 1) * plan["version_stride"]
    n = len(models)
    schedule = [
        [first + i, j, j if a.league else (i + j) % n] for i in range(count) for j in range(n)
    ]
    if a.mode == "historical":
        models.append(read(prior))
        schedule = [[13300014, 1, 0]]
    cfg = dict(
        root=str(ROOT),
        models=models,
        cases=len(schedule),
        schedule=schedule,
        protocol=plan,
        seconds=plan["seconds"],
        record=a.mode == "demo",
        study=a.mode == "train",
        press=a.mode != "historical",
        webots_sha256=sha(a.webots),
    )
    save(directory / "launch.json", cfg)
    if a.mode in ("qualify", "control", "historical"):
        with ThreadPoolExecutor(max_workers=2) as pool:
            list(pool.map(lambda i: isolated(directory, i, a.webots), range(len(schedule))))
    else:
        run_matches(directory, cfg, a.webots, WORLD, False)
    games = [read(directory / f"game-{i:03}.json") for i in range(len(schedule))]
    rows = [g["opening"] for g in games]
    if a.mode == "train":
        stats = []
        for j in range(n):
            owned = [
                r
                for r, g in zip(rows, games)
                if (t := r.get("chain", {}).get("team")) and g[t + "_model"] == j
            ]
            stats.append(totals(owned))
        best = max(
            range(n),
            key=lambda j: (stats[j]["goals"], stats[j]["shields"], -stats[j]["interceptions"]),
        )
        save(directory / "policy.json", models[best])
        save(directory / "returns.json", stats)
        archive = dict(
            config={k: v for k, v in cfg.items() if k != "root"},
            games=games,
            returns=stats,
            selected=best,
        )
        (directory / "training.json.gz").write_bytes(
            gzip.compress(json.dumps(archive, separators=(",", ":")).encode(), mtime=0)
        )
        print(stats)
    else:
        for i, (g, expected) in enumerate(zip(games, schedule)):
            states = np.load(directory / f"trace-{i:03}.npz")["states"]
            assert len(states) == 100 * cfg["seconds"] and g["seed"] == expected[0]
            assert pressure_result(states, g, plan) == rows[i]
        result = totals(rows)
        result.update(
            contacts=sum(g["stats"]["contacts"] for g in games),
            exits=sum(g["stats"]["exits"] for g in games),
        )
        if a.mode == "qualify":
            result["passed"] = len(rows) == 40 and result["goals"] >= plan["goal_target"]
        save(directory / "audit.json", result | dict(rows=rows, passing_goals=result["goals"]))
        print(result)


if __name__ == "__main__":
    main()
