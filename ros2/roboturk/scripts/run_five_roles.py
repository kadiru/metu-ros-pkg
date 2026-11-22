"""Run or watch one seeded, native 5v5 match and save its role trace."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "src/roboturk_ros/webots/worlds/ssl_five_roles.wbt"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output", type=Path)
    p.add_argument(
        "--webots",
        type=Path,
        default=shutil.which("webots")
        or ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots",
    )
    p.add_argument("--seed", type=int, default=20300000)
    p.add_argument("--seconds", type=int, default=60)
    p.add_argument("--referee", choices=("local", "ssl"), default="local")
    p.add_argument("--referee-address", default="224.5.23.1")
    p.add_argument("--referee-port", type=int, default=10003)
    p.add_argument("--referee-interface", default="0.0.0.0")
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--watch", action="store_true")
    mode.add_argument("--record", action="store_true")
    a = p.parse_args()
    if a.seconds < 1:
        p.error("Use a positive duration")
    dest = a.output.resolve()
    dest.mkdir(parents=True, exist_ok=False)
    (dest / "config.json").write_text(
        json.dumps(
            dict(
                root=str(ROOT),
                seed=a.seed,
                seconds=a.seconds,
                record=a.record,
                referee=a.referee,
                referee_address=a.referee_address,
                referee_port=a.referee_port,
                referee_interface=a.referee_interface,
            ),
            indent=2,
        )
        + "\n"
    )
    env = dict(
        os.environ,
        FIVE_ROLE_RUN=str(dest),
        PYTHONPATH=str(ROOT / "src/roboturk_core") + os.pathsep + os.environ.get("PYTHONPATH", ""),
        PATH=str(Path(sys.executable).parent) + os.pathsep + os.environ["PATH"],
    )
    cmd = [
        str(a.webots),
        "--batch",
        "--stdout",
        "--stderr",
        "--mode=realtime" if a.watch or a.referee == "ssl" else "--mode=fast",
    ]
    if not (a.watch or a.record):
        cmd.extend(["--no-rendering", "--minimize"])
    with (dest / "webots.log").open("w") as log:
        subprocess.run(
            [*cmd, str(WORLD)], env=env, stdout=log, stderr=subprocess.STDOUT, check=True
        )
    if not (dest / "positions.npz").exists():
        raise RuntimeError(f"Missing telemetry. See {dest}/webots.log")


if __name__ == "__main__":
    main()
