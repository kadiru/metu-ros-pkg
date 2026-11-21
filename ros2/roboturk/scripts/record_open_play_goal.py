#!/usr/bin/env python3
"""Record a qualified ordinary game using the frozen controller's ordinary demo slot."""

import argparse
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
from run_recovery_attack import R, ROOT, WORLD, read, save


def main():
    p = argparse.ArgumentParser()
    p.add_argument("qualification", type=Path)
    p.add_argument("folder", type=Path)
    p.add_argument(
        "--webots",
        type=Path,
        default=ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots",
    )
    a = p.parse_args()
    assert read(a.qualification / "audit.json")["passed"]
    for manifest in (
        R / "freeze.json",
        ROOT / "policies/contest_outlet_v1/freeze.json",
        ROOT / "policies/contest_outlet_v1/isolation-freeze.json",
        ROOT / "policies/contest_outlet_v2/freeze.json",
    ):
        for path, sha in read(manifest).items():
            assert hashlib.sha256((ROOT / path).read_bytes()).hexdigest() == sha
    games = []
    for f in sorted((a.qualification / "ordinary").glob("game-*.json")):
        g = read(f)
        for e in g["conversions"]:
            if e.get("passes"):
                games.append(((e["dribble_m"] >= 0.18, len(e["passes"]), -e["goal"]), g))
    _, game = max(games, key=lambda v: v[0])
    folder = a.folder.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    native = folder / "native"
    native.mkdir()
    meta = dict(
        root=str(ROOT),
        model=str(R / "policy.json"),
        seconds=game["seconds"],
        webots_sha256=hashlib.sha256(a.webots.read_bytes()).hexdigest(),
    )
    # In the frozen recording controller demo slot 2 is ordinary open play.
    # Running that single slot in a fresh process reproduces qualification.
    launch = dict(**meta, mode="demo", first=game["seed"] - 2, cases=3)
    save(native / "launch.json", launch)
    env = dict(
        os.environ,
        ATTACK_RUN=str(native),
        RECOVERY_CASE="2",
        PYTHONPATH=str(ROOT / "src/roboturk_core"),
        PATH=str(Path(sys.executable).parent) + os.pathsep + os.environ["PATH"],
    )
    with (native / "webots.log").open("w") as stream:
        subprocess.run(
            [str(a.webots.resolve()), "--batch", "--stdout", "--stderr", "--mode=fast", str(WORLD)],
            env=env,
            stdout=stream,
            stderr=subprocess.STDOUT,
            check=True,
        )
    for src, dst in (
        ("game-002.json", "game-000.json"),
        ("trace-002.npz", "trace-000.npz"),
        ("match.mp4", "match.mp4"),
        ("webots.log", "webots.log"),
        ("launch.json", "native-launch.json"),
    ):
        shutil.copyfile(native / src, folder / dst)
    save(
        folder / "launch.json",
        dict(
            **meta,
            mode="showcase",
            first=game["seed"],
            cases=1,
            selected_from="ordinary",
            native_slot=2,
        ),
    )


if __name__ == "__main__":
    main()
