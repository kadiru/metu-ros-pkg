#!/usr/bin/env python3
"""Select a post-reception carry by paired native rollout returns against a closing marker."""

import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--webots", required=True)
    parser.add_argument("--seed", type=int, default=33810000)
    parser.add_argument("--episodes", type=int, default=8)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--fit-first-touch", type=Path, help="Fit the carry start time for a frozen carry profile"
    )
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    candidates = []
    profiles = [
        dict(speed=speed, avoidance=weight)
        for speed, weight in [(0.95, 0.65), (1.25, 0.0), (1.25, 0.85), (1.5, 1.4)]
    ]
    if args.fit_first_touch:
        base = json.loads(args.fit_first_touch.read_text())
        profiles = [base, *(base | {"first_touch_steps": n} for n in (1, 4, 10))]
    for i, params in enumerate(profiles):
        profile = args.output / f"profile-{i}.json"
        profile.write_text(json.dumps(params, indent=2) + "\n")
        output = args.output / f"trials-{i}.json"
        subprocess.run(
            [
                sys.executable,
                "scripts/run_webots.py",
                "--webots",
                args.webots,
                "--mode",
                "team-closing-evaluate",
                "--carry-profile",
                str(profile),
                "--checkpoint",
                "policies/carpet_receive_v1/policy.zip",
                "--shoot-checkpoint",
                "policies/capture_dribble_shoot_v2/policy.zip",
                "--seed",
                str(args.seed),
                "--episodes",
                str(args.episodes),
                "--output",
                str(output),
                "--timeout",
                "1800",
            ],
            cwd=root,
            check=True,
        )
        data = json.loads(output.read_text())
        rows = data["episodes"]
        if [row["seed"] for row in rows] != list(range(args.seed, args.seed + args.episodes)):
            raise ValueError("Incomplete paired training cohort")
        collisions = sum(
            (row["shooting"] or row["transfer"])["defender_robot_contacts"] > 0 for row in rows
        )
        progress = sum((row["shooting"] or {}).get("carried_progress", 0) for row in rows)
        candidates.append(
            dict(
                profile=params,
                goals=data["successes"],
                collisions=collisions,
                loss_trials=data["possession_loss_episodes"],
                carry_progress=progress,
            )
        )

    def rank(c):
        return c["goals"], -c["collisions"], -c["loss_trials"], c["carry_progress"]

    best = max(candidates, key=rank)
    if args.fit_first_touch:
        challenger = max((c for c in candidates if "first_touch_steps" in c["profile"]), key=rank)
        (args.output / "challenger.json").write_text(
            json.dumps(challenger["profile"], indent=2) + "\n"
        )
    (args.output / "policy.json").write_text(json.dumps(best["profile"], indent=2) + "\n")
    (args.output / "selection.json").write_text(
        json.dumps(
            {
                "first_seed": args.seed,
                "episodes_per_profile": args.episodes,
                "candidates": candidates,
                "selected": best["profile"],
            },
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
