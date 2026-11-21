"""Native return-based search over first-touch and attacking decisions, followed by frozen testing."""

import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from audit_recovery_attack import panel
from run_recovery_attack import ROOT, read, save
from run_shield_attack import verify

R = ROOT / "policies/recovery_attack_v3"
WORLD = ROOT / "src/roboturk_ros/webots/worlds/ssl_transition.wbt"
CONTROLLER = ROOT / "src/roboturk_ros/webots/controllers/roboturk_transition/roboturk_transition.py"
ENGINE = ROOT / "src/roboturk_core/roboturk/transition_attack.py"


def initial():
    defaults = dict(
        tracking=0.48,
        centering=14.0,
        turn=4.0,
        escape=0.55,
        speed=0.95,
        pass_bias=-0.82,
        carry_bias=0.72,
        shot_bias=0.67,
    )
    variations = [
        {},
        dict(tracking=0.72, turn=3.0, escape=0.30, shot_bias=0.95),
        dict(tracking=0.34, turn=3.0, escape=0.85, carry_bias=0.92),
        dict(tracking=0.55, turn=5.5, escape=0.35, shot_bias=0.98),
        dict(tracking=0.66, turn=3.5, escape=0.95, carry_bias=1.24, pass_bias=-0.45),
        dict(tracking=0.76, turn=4.0, escape=0.6, carry_bias=1.05, speed=1.30, shot_bias=0.90),
    ]
    return [defaults | v for v in variations]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sources():
    return [
        ENGINE,
        CONTROLLER,
        WORLD,
        Path(__file__).resolve(),
        ROOT / "scripts/audit_recovery_attack.py",
        R / "protocol.json",
        ROOT / "policies/recovery_attack_v2/freeze.json",
    ]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=["search", "develop", "qualify", "demo", "branches"])
    p.add_argument("folder", type=Path)
    p.add_argument("--policy", type=Path)
    p.add_argument("--first", type=int)
    p.add_argument("--cases", type=int, default=48)
    p.add_argument("--candidates", type=Path)
    p.add_argument("--version", type=int, default=1)
    p.add_argument(
        "--webots",
        type=Path,
        default=ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots",
    )
    a = p.parse_args()
    if a.version < 1 or a.cases < 1:
        p.error("version and cases must be positive")
    verify()
    for name, sha in read(ROOT / "policies/recovery_attack_v2/freeze.json").items():
        assert digest(ROOT / name) == sha, name
    plan = read(R / "protocol.json")
    default = {
        "search": "training_first",
        "branches": "training_first",
        "develop": "development_first",
        "qualify": "qualification_first",
        "demo": "demo_first",
    }[a.mode]
    first = plan[default] + (a.version - 1) * 1000 if a.first is None else a.first
    if a.mode in ("qualify", "demo"):
        if a.first is not None or a.candidates is not None or a.policy is None:
            p.error("qualification and demo require the frozen policy and registered seeds")
        assert a.policy.resolve() == R / f"candidate-{a.version}.json"
        frozen = read(R / f"freeze-{a.version}.json")
        for f, h in frozen.items():
            assert digest(ROOT / f) == h, f
    models = read(a.candidates) if a.candidates else [read(a.policy)] if a.policy else initial()
    count = plan["cases"] if a.mode == "qualify" else 3 if a.mode == "demo" else a.cases
    if a.mode == "search":
        count *= len(models)
    elif a.mode == "branches":
        count *= 3
    d = a.folder.resolve()
    d.mkdir(parents=True, exist_ok=False)
    mode = {"search": "motors", "qualify": "trained", "develop": "trained"}.get(a.mode, a.mode)
    cfg = dict(
        root=str(ROOT),
        mode=mode,
        first=first,
        cases=count,
        seconds=35 if a.mode == "demo" else plan["seconds"],
        record=a.mode == "demo",
        candidates=models,
        webots_sha256=digest(a.webots),
    )
    save(d / "launch.json", cfg)
    env = dict(
        os.environ,
        ATTACK_RUN=str(d),
        PYTHONPATH=str(ROOT / "src/roboturk_core"),
        PATH=str(Path(sys.executable).parent) + os.pathsep + os.environ["PATH"],
    )
    cmd = [str(a.webots), "--batch", "--stdout", "--stderr", "--mode=fast"]
    if not cfg["record"]:
        cmd += ["--no-rendering", "--minimize"]
    isolated = a.mode in ("qualify", "develop")
    for i in range(count if isolated else 1):
        if isolated:
            env["RECOVERY_CASE"] = str(i)
        with (d / "webots.log").open("a") as f:
            subprocess.run(
                [*cmd, str(WORLD)], env=env, stdout=f, stderr=subprocess.STDOUT, check=True
            )
        assert (
            not (d / "error.txt").exists()
            and (d / f"game-{i if isolated else count - 1:03}.json").exists()
        ), d
    if a.mode in ("search", "branches"):
        games = [read(f) for f in sorted(d.glob("game-*.json"))]
        stats = []
        for j, m in enumerate(models):
            group = [g for g in games if g["control"] == j]
            won = [
                g["conversions"][0]
                for g in group
                if g["conversions"] and "goal" in g["conversions"][0]
            ]
            stats.append(
                dict(
                    index=j,
                    games=len(group),
                    goals=len(won),
                    passes=sum(bool(e["passes"]) for e in won),
                    carries=sum(e["dribble_m"] >= 0.18 for e in won),
                )
            )
        best = max(stats, key=lambda x: (x["goals"], x["passes"], x["carries"]))
        save(d / "returns.json", stats)
        save(d / "policy.json", models[best["index"]])
        (d / "training.json.gz").write_bytes(
            gzip.compress(
                json.dumps(
                    dict(launch=cfg, games=games, returns=stats), separators=(",", ":")
                ).encode(),
                mtime=0,
            )
        )
        print(stats)
    else:
        evidence = panel(d)
        evidence["passed"] = (
            evidence["totals"]["first"] >= plan["target"]
            if a.mode == "qualify"
            else evidence["totals"]["chains"] > 0
        )
        save(d / "audit.json", evidence)
        print(evidence["totals"], flush=True)


if __name__ == "__main__":
    main()
