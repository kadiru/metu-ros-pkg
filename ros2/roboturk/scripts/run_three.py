#!/usr/bin/env python3
"""Run a bounded six-robot native Webots experiment."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--webots", required=True, type=Path)
    p.add_argument(
        "--mode", required=True, choices=["train", "develop", "qualify", "demo", "control"]
    )
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--seed", type=int, default=901)
    p.add_argument("--first", type=int, default=7300000)
    p.add_argument("--matches", type=int, default=4)
    p.add_argument("--seconds", type=int, default=60)
    p.add_argument("--models", type=Path)
    args = p.parse_args()
    output = args.output.resolve()
    config = {
        **vars(args),
        "output": str(output),
        "root": str(ROOT),
        "models": str(args.models.resolve()) if args.models else None,
        "webots": str(args.webots.resolve()),
    }
    output.mkdir(parents=True, exist_ok=False)
    (output / "launch.json").write_text(json.dumps(config, indent=2))
    env = os.environ.copy()
    env.update(
        THREE_RUN=str(output),
        PYTHONPATH=str(ROOT / "src/roboturk_core"),
        PATH=str(Path(sys.executable).parent) + os.pathsep + env["PATH"],
    )
    cmd = [config["webots"], "--batch", "--stdout", "--stderr", "--mode=fast"]
    if args.mode != "demo":
        cmd += ["--no-rendering", "--minimize"]
    world = ROOT / "src/roboturk_ros/webots/worlds/ssl_three.wbt"
    with (output / "webots.log").open("w") as log:
        subprocess.run(
            cmd + [str(world)], env=env, stdout=log, stderr=subprocess.STDOUT, check=True
        )
    if not (output / "results.json").exists():
        raise RuntimeError(f"Controller produced no results. See {output}/webots.log")


if __name__ == "__main__":
    main()
