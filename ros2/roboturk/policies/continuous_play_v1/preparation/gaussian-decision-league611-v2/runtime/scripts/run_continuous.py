#!/usr/bin/env python3
"""Launch the isolated continuous-match world without changing prior frozen runtimes."""

import argparse
import hashlib
import json
from datetime import datetime
from zoneinfo import ZoneInfo
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--webots", required=True)
    p.add_argument(
        "--mode",
        choices=["pilot", "collect", "fit", "evaluate", "train", "demo", "contact-preflight"],
        required=True,
    )
    p.add_argument("--output", required=True)
    p.add_argument("--checkpoint")
    p.add_argument("--opponent")
    p.add_argument("--freeze", help="Verify a published runtime/model freeze before a reserved run")
    p.add_argument("--seed", type=int, default=4100000)
    p.add_argument("--episodes", type=int, default=12)
    p.add_argument("--duration", type=float, default=45)
    p.add_argument("--steps", type=int, default=32768)
    p.add_argument("--epochs", type=int, default=80)
    p.add_argument("--rounds", type=int, default=2)
    p.add_argument("--decisions-only", action="store_true")
    p.add_argument("--mix", type=float, default=0)
    p.add_argument("--timeout", type=float, default=3600)
    p.add_argument("--render", action="store_true")
    a = p.parse_args()
    if a.mode == "demo" and not a.render:
        p.error("Native movie recording requires --render")
    if min(a.episodes, a.duration, a.steps, a.epochs, a.rounds, a.timeout) <= 0:
        p.error("Positive limits required")
    root = Path(__file__).resolve().parents[1]
    output = Path(a.output)
    if a.mode in ("evaluate", "demo", "pilot", "collect", "contact-preflight") and (
        output.exists()
        or output.with_suffix(".json").exists()
        or output.with_suffix(".launch.json").exists()
    ):
        p.error("Existing experiment output must be retained; choose a new output path")
    if a.seed >= 4500000 and a.mode in ("evaluate", "demo") and not a.freeze:
        p.error("Reserved evaluation requires --freeze")
    if a.freeze:
        frozen_path = Path(a.freeze).resolve()
        frozen = json.loads(frozen_path.read_text())
        publication = json.loads((frozen_path.parent / "freeze-publication.json").read_text())
        if publication["freeze_sha256"] != hashlib.sha256(frozen_path.read_bytes()).hexdigest():
            p.error("Published freeze digest differs")
        if not publication.get("remote_hashes_verified") or not publication.get("commit"):
            p.error("Freeze has not been verified on GitHub")
        for relative, expected in frozen["runtime_sha256"].items():
            if hashlib.sha256((root / relative).read_bytes()).hexdigest() != expected:
                p.error("Frozen runtime changed: " + relative)
        for relative, expected in frozen["artifact_sha256"].items():
            if hashlib.sha256((frozen_path.parent / relative).read_bytes()).hexdigest() != expected:
                p.error("Frozen artifact changed: " + relative)
        for checkpoint in (a.checkpoint, a.opponent or a.checkpoint):
            relative = str(Path(checkpoint).resolve().relative_to(frozen_path.parent))
            if relative not in frozen["artifact_sha256"]:
                p.error("Checkpoint outside published selection")
    env = dict(os.environ)
    env["PATH"] = str(Path(sys.executable).parent) + os.pathsep + env["PATH"]
    env["PYTHONPATH"] = os.pathsep.join(
        [str(root / "src/roboturk_core"), env.get("PYTHONPATH", "")]
    )
    for k, v in vars(a).items():
        if v is not None:
            env["CONTINUOUS_" + k.upper()] = (
                str(Path(v).resolve())
                if k in ("output", "checkpoint", "opponent", "freeze")
                else str(v)
            )
    cmd = [a.webots, "--batch", "--stdout", "--stderr", "--mode=fast"]
    if not a.render:
        cmd += ["--no-rendering", "--minimize"]
    cmd += [str(root / "src/roboturk_ros/webots/worlds/ssl_continuous.wbt")]
    sources = list((root / "src/roboturk_core/roboturk").glob("continuous*.py"))
    sources += list((root / "src/roboturk_ros/webots/controllers/roboturk_continuous").glob("*.py"))
    sources += [
        Path(__file__).resolve(),
        root / "src/roboturk_ros/webots/worlds/ssl_continuous.wbt",
    ]
    receipt = Path(a.output).with_suffix(".launch.json")
    receipt.parent.mkdir(parents=True, exist_ok=True)
    receipt.write_text(
        json.dumps(
            {
                "started_at": datetime.now(ZoneInfo("America/Los_Angeles")).isoformat(),
                "arguments": vars(a),
                "runtime_sha256": {
                    str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in sources
                },
            },
            indent=2,
        )
        + "\n"
    )
    for source in sources:
        target = receipt.with_suffix("") / "runtime" / source.relative_to(root)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    child = subprocess.Popen(cmd, env=env, start_new_session=True)
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
