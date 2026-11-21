#!/usr/bin/env python3
"""Run the entire matched evaluation with one new native world per game."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

from run_contest import RELEASE, ROOT


def read(path):
    return json.loads(path.read_text())


def save(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--webots",
        type=Path,
        default=ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots",
    )
    args = parser.parse_args()
    protocol = read(RELEASE / "protocol.json")
    for name in ("freeze.json", "isolation-freeze.json"):
        for path, digest in read(RELEASE / name).items():
            assert hashlib.sha256((ROOT / path).read_bytes()).hexdigest() == digest, path
    world = ROOT / "src/roboturk_ros/webots/worlds/ssl_isolated_contest.wbt"
    args.output.mkdir(parents=True, exist_ok=False)
    count = protocol["matches_per_panel"]
    folders = {}
    for mode in protocol["panels"]:
        folder = (args.output / mode).resolve()
        folder.mkdir()
        folders[mode] = folder
        save(
            folder / "launch.json",
            dict(
                mode=mode,
                first=protocol["reserved_first"],
                matches=count,
                seconds=protocol["seconds"],
                root=str(ROOT),
                model=str(RELEASE / "policy.json"),
                process_per_match=True,
                webots_sha256=hashlib.sha256(args.webots.read_bytes()).hexdigest(),
            ),
        )
    for index in range(count):
        for mode, folder in folders.items():
            env = dict(
                os.environ,
                CONTEST_RUN=str(folder),
                CONTEST_INDEX=str(index),
                PYTHONPATH=str(ROOT / "src/roboturk_core"),
                PATH=str(Path(sys.executable).parent) + os.pathsep + os.environ["PATH"],
            )
            cmd = [
                str(args.webots.resolve()),
                "--batch",
                "--stdout",
                "--stderr",
                "--mode=fast",
                "--no-rendering",
                "--minimize",
                str(world),
            ]
            with (folder / "webots.log").open("a") as log:
                subprocess.run(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
            game = read(folder / f"game-{index:03}.json")
            print(index, mode, game["contest_metrics"]["chains"], flush=True)
    for folder in folders.values():
        save(
            folder / "results.json",
            [read(folder / f"game-{index:03}.json")["contest_metrics"] for index in range(count)],
        )
    subprocess.run(
        [sys.executable, str(ROOT / "scripts/audit_contest.py"), str(args.output.resolve())],
        check=True,
        env=dict(os.environ, PYTHONPATH=str(ROOT / "src/roboturk_core")),
    )


if __name__ == "__main__":
    main()
