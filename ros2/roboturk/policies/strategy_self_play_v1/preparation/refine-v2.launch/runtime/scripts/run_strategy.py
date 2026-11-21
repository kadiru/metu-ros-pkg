#!/usr/bin/env python3
"""Single-owner native learned strategy experiments, with immutable launch receipts."""

import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--webots", required=True)
    p.add_argument(
        "--mode",
        choices=("pilot", "collect", "prepare", "refine", "train", "evaluate", "demo"),
        required=True,
    )
    p.add_argument("--opponent")
    p.add_argument("--duration", type=float, default=20)
    p.add_argument("--mix", type=float, default=0)
    p.add_argument("--rounds", type=int, default=2)
    p.add_argument("--training-seed", type=int, default=811)
    p.add_argument("--output", required=True)
    p.add_argument("--checkpoint")
    p.add_argument("--episodes", type=int, default=20)
    p.add_argument("--seed", type=int, default=6100000)
    p.add_argument("--steps", type=int, default=32768)
    p.add_argument("--timeout", type=float, default=7200)
    p.add_argument("--render", action="store_true")
    p.add_argument("--freeze")
    a = p.parse_args()
    output = Path(a.output).resolve()
    receipt = output.with_suffix(".launch.json")
    if output.exists() or output.with_suffix(".json").exists() or receipt.exists():
        p.error("Retain existing attempts; choose a new output path")
    if a.mode == "demo" and not a.render:
        p.error("Native recording requires --render")
    if a.mode in ("evaluate", "demo") and a.seed >= 6500000 and not a.freeze:
        p.error("Reserved cases require a published freeze")
    if a.freeze:
        freeze = Path(a.freeze).resolve()
        f = json.loads(freeze.read_text())
        pub = json.loads((freeze.parent / "freeze-publication.json").read_text())
        assert (
            pub["remote_hashes_verified"]
            and pub["freeze_sha256"] == hashlib.sha256(freeze.read_bytes()).hexdigest()
        )
        for rel, h in f["files"].items():
            assert hashlib.sha256((ROOT / rel).read_bytes()).hexdigest() == h, rel
    env = dict(os.environ)
    env["PATH"] = str(Path(sys.executable).parent) + os.pathsep + env["PATH"]
    env["PYTHONPATH"] = str(ROOT / "src/roboturk_core") + os.pathsep + env.get("PYTHONPATH", "")
    for k, v in vars(a).items():
        if v is not None:
            env["STRATEGY_" + k.upper()] = (
                str(Path(v).resolve())
                if k in ("output", "checkpoint", "opponent", "freeze")
                else str(v)
            )
    sources = list((ROOT / "src/roboturk_core/roboturk").glob("*.py"))
    sources += list((ROOT / "src/roboturk_ros/webots/controllers/roboturk_strategy").glob("*.py"))
    sources += list((ROOT / "src/roboturk_ros/webots/protos").glob("*.proto"))
    sources += [Path(__file__).resolve(), ROOT / "src/roboturk_ros/webots/worlds/ssl_strategy.wbt"]
    receipt.parent.mkdir(parents=True, exist_ok=True)
    receipt.write_text(
        json.dumps(
            {
                "started_at": datetime.now(ZoneInfo("America/Los_Angeles")).isoformat(),
                "arguments": vars(a),
                "webots_executable_sha256": hashlib.sha256(Path(a.webots).read_bytes()).hexdigest(),
                "runtime_sha256": {
                    str(s.relative_to(ROOT)): hashlib.sha256(s.read_bytes()).hexdigest()
                    for s in sources
                },
            },
            indent=2,
        )
        + "\n"
    )
    for source in sources:
        target = receipt.with_suffix("") / "runtime" / source.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    command = [a.webots, "--batch", "--stdout", "--stderr", "--mode=fast"]
    if not a.render:
        command += ["--no-rendering", "--minimize"]
    command += [str(ROOT / "src/roboturk_ros/webots/worlds/ssl_strategy.wbt")]
    child = subprocess.Popen(command, env=env, start_new_session=True)
    try:
        return child.wait(timeout=a.timeout)
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        os.killpg(child.pid, signal.SIGTERM)
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()
        return 124


if __name__ == "__main__":
    raise SystemExit(main())
