#!/usr/bin/env python3
"""Launch native self-play training, development, matched evaluation or a recording."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

from run_contest import ROOT

R = ROOT / "policies/recovery_attack_v1"
WORLD = ROOT / "src/roboturk_ros/webots/worlds/ssl_recovery_attack.wbt"


def read(p):
    return json.loads(p.read_text())


def save(p, data):
    p.write_text(json.dumps(data, indent=2) + "\n")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=["train", "develop", "qualify", "demo"])
    p.add_argument("folder", type=Path)
    p.add_argument("--model", type=Path, default=R / "policy.json")
    p.add_argument("--cases", type=int)
    p.add_argument("--offset", type=int, default=0)
    p.add_argument(
        "--webots",
        type=Path,
        default=ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots",
    )
    a = p.parse_args()
    plan = read(R / "protocol.json")
    folder = a.folder.resolve()
    for path in [
        ROOT / "policies/contest_outlet_v1/freeze.json",
        ROOT / "policies/contest_outlet_v1/isolation-freeze.json",
        ROOT / "policies/contest_outlet_v2/freeze.json",
    ]:
        for f, digest in read(path).items():
            assert hashlib.sha256((ROOT / f).read_bytes()).hexdigest() == digest, f
    if a.mode in ("qualify", "demo"):
        assert a.cases is None and a.offset == 0 and a.model.resolve() == R / "policy.json"
        for f, digest in read(R / "freeze.json").items():
            assert hashlib.sha256((ROOT / f).read_bytes()).hexdigest() == digest, f
    assert 0 <= a.offset < 50000
    binary = a.webots.resolve()
    meta = dict(
        root=str(ROOT),
        model=str(a.model.resolve()),
        webots_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
    )

    def simulate(path, idx=None):
        env = dict(
            os.environ,
            ATTACK_RUN=str(path),
            PYTHONPATH=str(ROOT / "src/roboturk_core"),
            PATH=str(Path(sys.executable).parent) + os.pathsep + os.environ["PATH"],
        )
        if idx is not None:
            env["RECOVERY_CASE"] = str(idx)
        cmd = [str(binary), "--batch", "--stdout", "--stderr", "--mode=fast"]
        if a.mode != "demo":
            cmd += ["--no-rendering", "--minimize"]
        with (path / "webots.log").open("a") as stream:
            subprocess.run(
                [*cmd, str(WORLD)], env=env, stdout=stream, stderr=subprocess.STDOUT, check=True
            )
        require = idx if idx is not None else read(path / "launch.json")["cases"] - 1
        if not (path / f"game-{require:03}.json").exists() or (path / "error.txt").exists():
            raise RuntimeError(f"Native game missing in {path}")

    folder.mkdir(parents=True, exist_ok=False)
    if a.mode != "qualify":
        n = a.cases or (
            144 if a.mode == "train" else plan["demo_cases"] if a.mode == "demo" else 16
        )
        first = (
            plan[
                {"train": "train_first", "develop": "development_first", "demo": "demo_first"}[
                    a.mode
                ]
            ]
            + a.offset
        )
        assert 0 < n <= 1000
        save(
            folder / "launch.json",
            dict(
                **meta,
                mode=a.mode,
                first=first,
                cases=n,
                seconds=plan["demo_seconds" if a.mode == "demo" else "seconds"],
            ),
        )
        simulate(folder)
        return
    panels = {
        "released": plan["cases"],
        "trained": plan["cases"],
        "mixed": plan["mixed_cases"],
        "ordinary": plan["ordinary_cases"],
    }
    for mode, n in panels.items():
        path = folder / mode
        path.mkdir()
        save(
            path / "launch.json",
            dict(
                **meta,
                mode=mode,
                first=plan["ordinary_first" if mode == "ordinary" else "reserved_first"],
                cases=n,
                seconds=plan["ordinary_seconds" if mode == "ordinary" else "seconds"],
            ),
        )
    for i in range(max(panels.values())):
        for mode, n in panels.items():
            if i < n:
                simulate(folder / mode, i)
                print(mode, i, flush=True)


if __name__ == "__main__":
    main()
