#!/usr/bin/env python3
"""Retained paired development and fresh cross-play, strictly before reserved tests."""

import argparse
import json
from pathlib import Path
import subprocess
import sys
from audit_strategy import audit

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--webots", required=True)
    p.add_argument("--league811", type=Path, required=True)
    p.add_argument("--league822", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--pass-motor", type=Path)
    p.add_argument("--paired-seed", type=int, default=6300200)
    p.add_argument("--cross-seed", type=int, default=6301000)
    a = p.parse_args()
    assert 6300000 <= a.paired_seed < 6399900 and 6300000 <= a.cross_seed < 6399800
    if a.output.exists():
        p.error("Retain previous development attempts")
    a.output.mkdir(parents=True)
    paths = {
        811: a.league811 / "generation2/policy.zip",
        822: a.league822 / "generation2/policy.zip",
    }
    cases = [
        ("811-paired", 811, 811, a.paired_seed),
        ("822-paired", 822, 822, a.paired_seed),
        ("811-cross", 811, 822, a.cross_seed),
        ("822-cross", 822, 811, a.cross_seed + 100),
    ]
    results = {}
    for name, blue, yellow, seed in cases:
        out = a.output / name
        command = [
            sys.executable,
            str(ROOT / "scripts/run_strategy.py"),
            "--webots",
            a.webots,
            "--mode",
            "evaluate",
            "--checkpoint",
            str(paths[blue]),
            "--opponent",
            str(paths[yellow]),
            "--output",
            str(out),
            "--episodes",
            "16",
            "--seed",
            str(seed),
            "--duration",
            "20",
            "--timeout",
            "3600",
        ]
        if a.pass_motor:
            command += ["--pass-motor", str(a.pass_motor)]
        with out.with_suffix(".log").open("w") as log:
            subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
        data = json.loads(out.with_suffix(".json").read_text())
        check = audit(out.with_suffix(".json"))
        out.with_suffix(".audit.json").write_text(json.dumps(check, indent=2) + "\n")
        results[name] = {
            "summary": data["summary"],
            "audit_passed": check["passed"],
            "selectors": data["selectors"],
        }
        (a.output / "development.json").write_text(json.dumps(results, indent=2) + "\n")
        print(name, results[name], flush=True)
    return 0 if all(r["audit_passed"] for r in results.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
