#!/usr/bin/env python3
"""Run a bounded six-robot native Webots experiment."""

import argparse
import hashlib
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
    p.add_argument("--first", type=int)
    p.add_argument("--matches", type=int)
    p.add_argument("--seconds", type=int, default=60)
    p.add_argument("--models", type=Path)
    args = p.parse_args()
    release = ROOT / "policies/recipient_3v3_v1"
    protocol = json.loads((release / "protocol.json").read_text())
    reserved = args.mode in ("qualify", "demo")
    plan = protocol["qualification" if args.mode == "qualify" else "demo"] if reserved else {}
    default_first = 7110000 + (args.seed - 901) * 100000 if args.mode == "train" else 7300000
    if args.first is None:
        args.first = plan.get("first", default_first)
    if args.matches is None:
        args.matches = plan.get("matches", 20 if args.mode == "train" else 4)
    if args.matches <= 0 or args.seconds <= 0:
        p.error("matches and seconds must be positive")
    if reserved and (args.first, args.matches, args.seconds) != (
        plan["first"],
        plan["matches"],
        protocol["seconds"],
    ):
        p.error("Use the complete reserved panel specified in protocol.json")
    if not reserved:
        low, high = (7100000, 7300000) if args.mode == "train" else (7300000, 7400000)
        if not (low <= args.first and args.first + args.matches <= high):
            p.error("Select cases from the declared training or development split")
    if args.mode != "train" and args.models is None:
        args.models = release / "selectors.json"
    if args.models:
        params = json.loads(args.models.read_text())
        for seed in ("901", "902"):
            if len(params[seed]) != 6 or not all(
                isinstance(x, (int, float)) and abs(x) < float("inf") for x in params[seed]
            ):
                p.error("Expected six finite recipient weights per policy")
    if reserved:
        freeze = json.loads((release / "freeze.json").read_text())
        for name, digest in freeze["files"].items():
            if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != digest:
                p.error("Frozen file differs: " + name)
        if args.models.resolve() != (release / "selectors.json").resolve():
            p.error("Reserved evaluation uses the frozen recipient selectors")
    output = args.output.resolve()
    config = {
        **vars(args),
        "output": str(output),
        "root": str(ROOT),
        "models_sha256": hashlib.sha256(args.models.read_bytes()).hexdigest()
        if args.models
        else None,
        "webots_sha256": hashlib.sha256(args.webots.read_bytes()).hexdigest(),
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
