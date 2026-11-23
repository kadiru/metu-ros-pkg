"""Run four paired two-half matches against the active referee in native Webots."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

from run_fixture import ROOT

SEEDS = range(28300000, 28300004)
ARMS = ("baseline", "steering")
KICKOFF_SEEDS = range(29300000, 29300004)
KICKOFF_ARMS = ("reset", "drive")
PARALLEL_SEEDS = range(30300000, 30300004)
PARALLEL_ARMS = ("current", "parallel")
TEAM_SEEDS = range(31300000, 31300004)
TEAM_ARMS = ("shared", "teams")
ACTOR = "policies/open_motion_v1/actor.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    comparison = parser.add_mutually_exclusive_group()
    comparison.add_argument(
        "--compare-parallel", action="store_true", help="Compare two driven kickoff planners"
    )
    comparison.add_argument(
        "--compare-kickoffs",
        action="store_true",
        help="Hold learned steering fixed and compare reset with driven formations",
    )
    comparison.add_argument(
        "--compare-teams",
        action="store_true",
        help="Compare shared and team-local kickoff planning",
    )
    options = ("game-controller", "distribution", "classes", "java", "webots")
    for option in options:
        parser.add_argument("--" + option, type=Path)
    args = parser.parse_args()
    runtime = []
    for option in options:
        value = getattr(args, option.replace("-", "_"))
        if value is not None:
            runtime.extend(["--" + option, str(value.resolve())])
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    seeds, arms = (KICKOFF_SEEDS, KICKOFF_ARMS) if args.compare_kickoffs else (SEEDS, ARMS)
    if args.compare_parallel:
        seeds, arms = PARALLEL_SEEDS, PARALLEL_ARMS
    if args.compare_teams:
        seeds, arms = TEAM_SEEDS, TEAM_ARMS
    for index, seed in enumerate(seeds):
        pair = root / str(seed)
        pair.mkdir()
        for arm in arms if index % 2 == 0 else reversed(arms):
            spec = dict(
                mode="league_match", seed=seed, half_seconds=30, seconds=240, film=index == 0
            )
            if args.compare_kickoffs:
                spec.update(drive_kickoffs=arm == "drive", track_kickoffs=True, seconds=360)
            if args.compare_parallel:
                spec.update(
                    drive_kickoffs=True,
                    parallel_kickoffs=arm == "parallel",
                    track_kickoffs=True,
                    seconds=360,
                )
            if args.compare_teams:
                spec.update(
                    drive_kickoffs=True,
                    parallel_kickoffs=arm == "shared",
                    team_kickoffs=arm == "teams",
                    track_kickoffs=True,
                    seconds=360,
                )
            if (
                args.compare_teams
                or args.compare_parallel
                or args.compare_kickoffs
                or arm == "steering"
            ):
                spec["open_play_policy"] = ACTOR
            file = pair / f"{arm}.json"
            file.write_text(json.dumps(spec, indent=2) + "\n")
            print(seed, arm, flush=True)
            subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts/autoref/run_placement.py"),
                    str(file),
                    str(pair / arm),
                    *runtime,
                ],
                check=True,
            )


if __name__ == "__main__":
    main()
