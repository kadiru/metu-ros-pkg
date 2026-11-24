"""Select Blue's finish by goals on common native Webots training openings."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

from compare_guarded_finish import compare, summarize


def save(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--webots", required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--seed", type=int, default=36010000)
    p.add_argument("--episodes", type=int, default=10)
    args = p.parse_args()
    root = args.output
    root.mkdir(exist_ok=True, parents=True)
    base = json.loads(Path("policies/secure_receive_v1/policy.json").read_text())
    profiles = [base]
    for distance, corner, prediction in (
        (0.70, 0.38, 0),
        (0.95, 0.42, 0.7),
        (1.30, 0.42, 0.7),
        (0.70, 0.44, 1),
        (1.15, 0.38, 0),
        (1.55, 0.44, 1),
    ):
        profiles.append(
            base
            | {"finish": dict(release_range=distance, corner_offset=corner, prediction=prediction)}
        )
    candidates = []
    reference = None
    for i, profile in enumerate(profiles):
        source, output = root / f"profile-{i}.json", root / f"arm-{i}.json"
        save(source, profile)
        subprocess.run(
            [
                sys.executable,
                "scripts/run_webots.py",
                "--webots",
                args.webots,
                "--mode",
                "team-guarded-evaluate",
                "--carry-profile",
                str(source),
                "--checkpoint",
                "policies/carpet_receive_v1/policy.zip",
                "--shoot-checkpoint",
                "policies/capture_dribble_shoot_v2/policy.zip",
                "--keeper-checkpoint",
                "policies/learned_opponents_v1/keeper/policy.zip",
                "--seed",
                str(args.seed),
                "--episodes",
                str(args.episodes),
                "--output",
                str(output),
                "--timeout",
                "1800",
            ],
            check=True,
        )
        data = json.loads(output.read_text())
        if [r["seed"] for r in data["episodes"]] != list(
            range(args.seed, args.seed + args.episodes)
        ):
            raise ValueError("Incomplete training cohort")
        if reference is None:
            reference = data
        compare(reference, data)
        stats = summarize(data)
        goal_seconds = sum(r["elapsed_seconds"] for r in data["episodes"] if r["is_success"])
        candidates.append(dict(profile=profile, metrics=stats, total_goal_seconds=goal_seconds))
        print(f"Arm {i}: {stats['receptions']} receptions, {stats['goals']} goals", flush=True)
    best = max(
        candidates,
        key=lambda c: (
            c["metrics"]["goals"],
            -c["metrics"]["outcomes"].get("lost_possession", 0),
            -c["total_goal_seconds"],
        ),
    )
    save(root / "policy.json", best["profile"])
    save(
        root / "selection.json",
        dict(seed=args.seed, episodes=args.episodes, candidates=candidates, selected=best),
    )


if __name__ == "__main__":
    main()
