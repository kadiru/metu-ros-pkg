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
ACTOR = "policies/open_motion_v1/actor.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--compare-kickoffs",
        action="store_true",
        help="Hold learned steering fixed and compare reset with driven formations",
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
    for index, seed in enumerate(seeds):
        pair = root / str(seed)
        pair.mkdir()
        for arm in arms if index % 2 == 0 else reversed(arms):
            spec = dict(
                mode="league_match", seed=seed, half_seconds=30, seconds=240, film=index == 0
            )
            if args.compare_kickoffs:
                spec.update(drive_kickoffs=arm == "drive", track_kickoffs=True, seconds=360)
            if args.compare_kickoffs or arm == "steering":
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
