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
            "decision-probe",
            "decision-bootstrap",
            "decision-prepare",
            "decision-commitment",
            "decision-panel",
            "decision-train",
            "decision-train-precision",
            "decision-repair",
            "decision-spacing",
            "decision-flank",
            "decision-evaluate",
            "decision-demo",
            "selfplay-probe",
            "selfplay-bootstrap",
            "selfplay-train",
            "selfplay-evaluate",
            "selfplay-panel",
            "selfplay-demo",
            "evaluate",
            "suite",
            "physics-check",
            "train",
            "ros",
            "capture-train",
            "capture-evaluate",
            "capture-demo",
            "dribble-train",
            "dribble-refine",
            "dribble-evaluate",
            "dribble-demo",
            "receive-dojo",
            "receive-dojo-demo",
            "opponents-train",
            "opponents-challenge",
            "opponents-evaluate",
            "opponents-attack-train",
            "opponents-finish-evaluate",
            "opponents-demo",
            "keeper-train",
            "keeper-finish-evaluate",
            "keeper-evaluate",
            "keeper-baseline",
            "keeper-demo",
            "micro-train",
            "micro-evaluate",
            "micro-demo",
            "defended-train",
            "defended-upgrade",
            "defended-tactical-train",
            "defended-evaluate",
            "defended-demo",
            "team-train",
            "team-upgrade",
            "team-fit-receiver",
            "team-evaluate",
            "team-demo",
            "team-closing-evaluate",
            "team-closing-demo",
            "team-guarded-evaluate",
            "team-guarded-demo",
            "shoot-train",
            "shoot-upgrade",
            "shoot-evaluate",
            "shoot-demo",
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
    parser.add_argument("--carry-profile", help="Closing-marker carry parameters (JSON)")
    parser.add_argument("--shoot-checkpoint", help="Frozen scoring policy for the two-robot play")
    parser.add_argument("--defender-checkpoint")
    parser.add_argument("--keeper-checkpoint")
    parser.add_argument("--timeout", type=float, default=120)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    if not args.webots:
        parser.error("Pass --webots /path/to/webots or add webots to PATH")
    if min(args.episodes, args.steps, args.timeout) <= 0:
        parser.error("episodes, steps and timeout must be positive")
    if args.mode == "receive-dojo-demo" and not args.checkpoint:
        parser.error("Pass the receive-dojo comparison JSON with --checkpoint")
    env = dict(os.environ)
    env["PATH"] = str(Path(sys.executable).parent) + os.pathsep + env["PATH"]
    env["PYTHONPATH"] = os.pathsep.join(
        [str(root / "src/roboturk_core"), str(root / "src/roboturk_ros"), env.get("PYTHONPATH", "")]
    )
    for name, value in vars(args).items():
        if name in ("mode", "skill", "episodes", "steps", "seed"):
            env["ROBOTURK_" + name.upper()] = str(value)
    if args.mode == "physics-check" and args.render:
        env["ROBOTURK_FILM_ROLL"] = "1"
    env["ROBOTURK_OUTPUT"] = str(Path(args.output).resolve())
    if args.mode.startswith("team-guarded-") and not all(
        (args.keeper_checkpoint, args.carry_profile, args.checkpoint, args.shoot_checkpoint)
    ):
        parser.error("A guarded team requires the keeper, transfer, scorer and carry profile")
    if args.carry_profile:
        if (
            not args.mode.startswith(("team-closing-", "team-guarded-"))
            or not args.shoot_checkpoint
        ):
            parser.error(
                "--carry-profile requires a team-closing or team-guarded mode and --shoot-checkpoint"
            )
        env["ROBOTURK_CARRY_PROFILE"] = str(Path(args.carry_profile).resolve())
    else:
        env.pop("ROBOTURK_CARRY_PROFILE", None)
    if args.checkpoint:
        env["ROBOTURK_CHECKPOINT"] = str(Path(args.checkpoint).resolve())
    if args.shoot_checkpoint:
        env["ROBOTURK_SHOOT_CHECKPOINT"] = str(Path(args.shoot_checkpoint).resolve())
    for role in ("defender", "keeper"):
        path = getattr(args, role + "_checkpoint")
        if path:
            env["ROBOTURK_" + role.upper() + "_CHECKPOINT"] = str(Path(path).resolve())
    command = [
        args.webots,
        "--batch",
        "--stdout",
        "--stderr",
        "--mode=realtime" if args.mode == "ros" else "--mode=fast",
    ]
    if not args.render and args.mode != "receive-dojo-demo":
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
