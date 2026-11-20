#!/usr/bin/env python3
"""Run all predeclared reserved panels once, serially, after a published freeze."""

import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--webots", required=True)
    p.add_argument("--release", type=Path, required=True)
    args = p.parse_args()
    release = args.release.resolve()
    freeze = release / "freeze.json"
    frozen = json.loads(freeze.read_text())
    models = {}
    for selection in frozen["selection"]:
        for label, index in [("initial", 0), ("final", -1)]:
            models[f"{selection['seed']}-{label}"] = release / selection["snapshots"][index]["path"]
    output = release / "qualification"
    output.mkdir(exist_ok=True)
    for panel in frozen["protocol"]["panels"]:
        destination = output / panel["name"]
        if destination.with_suffix(".launch.json").exists():
            raise RuntimeError(
                "Reserved attempt already exists; retain and investigate: " + panel["name"]
            )
        print("Reserved panel", panel["name"], "starting", flush=True)
        command = [
            sys.executable,
            str(Path(__file__).with_name("run_continuous.py")),
            "--webots",
            args.webots,
            "--mode",
            "evaluate",
            "--freeze",
            str(freeze),
            "--checkpoint",
            str(models[panel["blue"]]),
            "--opponent",
            str(models[panel["yellow"]]),
            "--episodes",
            str(panel["count"]),
            "--seed",
            str(panel["seed"]),
            "--duration",
            str(frozen["protocol"]["duration"]),
            "--output",
            str(destination),
            "--timeout",
            "1800",
        ]
        with destination.with_suffix(".log").open("x") as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        result = json.loads(destination.with_suffix(".json").read_text())
        print("Reserved panel", panel["name"], result["summary"], flush=True)
    subprocess.run(
        [
            sys.executable,
            str(Path(__file__).with_name("qualify_continuous.py")),
            "--release",
            str(release),
        ],
        check=True,
    )


if __name__ == "__main__":
    main()
