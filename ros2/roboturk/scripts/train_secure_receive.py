"""Fit approach speed, ball velocity matching and mouth feedback by paired Webots returns."""

import argparse
from itertools import product
import json
from pathlib import Path
import subprocess
import sys

from compare_closing_press import check_openings, summarize


def save(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--webots", required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--seed", type=int, default=34910000)
    p.add_argument("--episodes", type=int, default=12)
    args = p.parse_args()
    base = json.loads(Path("policies/closing_carry_v1/policy.json").read_text())
    root = args.output
    root.mkdir(parents=True, exist_ok=True)
    profiles = [base]
    for speed, cushion, gain in product((0.6, 1.3), (0.4, 0.9), (4.0, 10.0)):
        profiles.append(
            base | {"receive": dict(approach_speed=speed, cushion=cushion, settle_gain=gain)}
        )
    fits = []
    baseline = None
    for i, profile in enumerate(profiles):
        source = root / f"profile-{i}.json"
        output = root / f"arm-{i}.json"
        save(source, profile)
        subprocess.run(
            [
                sys.executable,
                "scripts/run_webots.py",
                "--webots",
                args.webots,
                "--mode",
                "team-closing-evaluate",
                "--checkpoint",
                "policies/carpet_receive_v1/policy.zip",
                "--shoot-checkpoint",
                "policies/capture_dribble_shoot_v2/policy.zip",
                "--carry-profile",
                str(source),
                "--seed",
                str(args.seed),
                "--episodes",
                str(args.episodes),
                "--timeout",
                "1800",
                "--output",
                str(output),
            ],
            check=True,
        )
        data = json.loads(output.read_text())
        if [r["seed"] for r in data["episodes"]] != list(
            range(args.seed, args.seed + args.episodes)
        ):
            raise ValueError("Incomplete training cohort")
        if baseline is None:
            baseline = data
        check_openings(baseline, data)
        metrics = summarize(data["episodes"])
        fits.append(dict(profile=profile, metrics=metrics))
        print(
            f"Arm {i}: {metrics['completed_passes']} receptions, {metrics['goals']} goals",
            flush=True,
        )
    winner = max(
        fits,
        key=lambda arm: (
            arm["metrics"]["completed_passes"],
            arm["metrics"]["goals"],
            -arm["metrics"]["marker_robot_proximity_trials"],
        ),
    )
    save(
        root / "selection.json",
        dict(first_seed=args.seed, count=args.episodes, candidates=fits, winner=winner),
    )
    save(root / "policy.json", winner["profile"])


if __name__ == "__main__":
    main()
