#!/usr/bin/env python3
"""Launch a serial native contested-possession experiment."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
RELEASE = ROOT / "policies/contest_outlet_v1"


def main():
    p = argparse.ArgumentParser()
    p.add_argument(
        "mode", choices=["train", "develop", "released", "trained", "head_to_head", "demo"]
    )
    p.add_argument("output", type=Path)
    p.add_argument(
        "--webots",
        type=Path,
        default=ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots",
    )
    p.add_argument("--first", type=int)
    p.add_argument("--matches", type=int)
    p.add_argument("--seconds", type=int)
    p.add_argument("--model", type=Path, default=RELEASE / "policy.json")
    p.add_argument("--seed", type=int, default=917)
    a = p.parse_args()
    protocol = json.loads((RELEASE / "protocol.json").read_text())
    if a.mode in ("train", "develop"):
        first = a.first or protocol["training_first" if a.mode == "train" else "development_first"]
        count = a.matches or (96 if a.mode == "train" else 12)
        seconds = a.seconds or 10
        lower = protocol["training_first" if a.mode == "train" else "development_first"]
        if not (lower <= first and first + count < lower + 100000) or min(count, seconds) < 1:
            p.error("Invalid training/development cases")
    else:
        if any(v is not None for v in (a.first, a.matches, a.seconds)):
            p.error("Use full published panel")
        demo = a.mode == "demo"
        first = protocol["demo_first" if demo else "reserved_first"]
        count = protocol["demo_matches" if demo else "matches_per_panel"]
        seconds = protocol["seconds"]
        f = json.loads((RELEASE / "freeze.json").read_text())
        for path, digest in f.items():
            assert hashlib.sha256((ROOT / path).read_bytes()).hexdigest() == digest, path
        assert a.model.resolve() == (RELEASE / "policy.json").resolve()
    output = a.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    config = dict(
        mode=a.mode,
        first=first,
        matches=count,
        seconds=seconds,
        root=str(ROOT),
        seed=a.seed,
        model=str(a.model.resolve()),
        webots_sha256=hashlib.sha256(a.webots.read_bytes()).hexdigest(),
    )
    (output / "launch.json").write_text(json.dumps(config, indent=2) + "\n")
    cmd = [str(a.webots.resolve()), "--batch", "--stdout", "--stderr", "--mode=fast"]
    if a.mode != "demo":
        cmd += ["--no-rendering", "--minimize"]
    cmd += [str(ROOT / "src/roboturk_ros/webots/worlds/ssl_contest.wbt")]
    env = dict(
        os.environ,
        CONTEST_RUN=str(output),
        PYTHONPATH=str(ROOT / "src/roboturk_core"),
        PATH=str(Path(sys.executable).parent) + os.pathsep + os.environ["PATH"],
    )
    with (output / "webots.log").open("w") as log:
        subprocess.run(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    if not (output / "results.json").exists():
        raise RuntimeError(f"Missing native results in {output}")


if __name__ == "__main__":
    main()
