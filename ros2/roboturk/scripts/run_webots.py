#!/usr/bin/env python3
"""Launch the source world using this interpreter's environment, with a bounded run."""

import argparse
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--webots", default=shutil.which("webots"))
    parser.add_argument(
        "--mode",
        choices=[
            "evaluate",
            "suite",
            "train",
            "ros",
            "capture-train",
            "capture-evaluate",
            "capture-demo",
            "dribble-train",
            "dribble-refine",
            "dribble-evaluate",
            "dribble-demo",
        ],
        default="evaluate",
    )
    parser.add_argument(
        "--skill", choices=["move", "chase", "dribble", "pass", "receive", "shoot"], default="chase"
    )
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--steps", type=int, default=2048)
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--output", default="runs/webots/result")
    parser.add_argument("--checkpoint")
    parser.add_argument("--timeout", type=float, default=120)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    if not args.webots:
        parser.error("Pass --webots /path/to/webots or add webots to PATH")
    if min(args.episodes, args.steps, args.timeout) <= 0:
        parser.error("episodes, steps and timeout must be positive")
    env = dict(os.environ)
    env["PATH"] = str(Path(sys.executable).parent) + os.pathsep + env["PATH"]
    env["PYTHONPATH"] = os.pathsep.join(
        [str(root / "src/roboturk_core"), str(root / "src/roboturk_ros"), env.get("PYTHONPATH", "")]
    )
    for name, value in vars(args).items():
        if name in ("mode", "skill", "episodes", "steps", "seed"):
            env["ROBOTURK_" + name.upper()] = str(value)
    env["ROBOTURK_OUTPUT"] = str(Path(args.output).resolve())
    if args.checkpoint:
        env["ROBOTURK_CHECKPOINT"] = str(Path(args.checkpoint).resolve())
    command = [
        args.webots,
        "--batch",
        "--stdout",
        "--stderr",
        "--mode=realtime" if args.mode == "ros" else "--mode=fast",
    ]
    if not args.render:
        command += ["--no-rendering", "--minimize"]
    command.append(str(root / "src/roboturk_ros/webots/worlds/ssl_training.wbt"))
    process = subprocess.Popen(command, env=env, start_new_session=True)
    try:
        return process.wait(timeout=args.timeout)
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        print("Stopped the owned Webots process group after timeout/interruption", file=sys.stderr)
        return 124


if __name__ == "__main__":
    raise SystemExit(main())
