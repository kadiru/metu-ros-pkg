"""Run four paired two-half matches against the active referee in native Webots."""

import argparse
import json
from pathlib import Path
import subprocess
import sys

from run_fixture import ROOT

SEEDS = range(28300000, 28300004)
ARMS = ("baseline", "steering")
ACTOR = "policies/open_motion_v1/actor.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    for option in ("game-controller", "distribution", "classes", "java", "webots"):
        parser.add_argument("--" + option, type=Path)
    args = parser.parse_args()
    runtime = []
    for key, value in vars(args).items():
        if key != "output" and value is not None:
            runtime.extend(["--" + key.replace("_", "-"), str(value.resolve())])
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    for index, seed in enumerate(SEEDS):
        pair = root / str(seed)
        pair.mkdir()
        for arm in ARMS if index % 2 == 0 else reversed(ARMS):
            spec = dict(
                mode="league_match", seed=seed, half_seconds=30, seconds=240, film=index == 0
            )
            if arm == "steering":
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
