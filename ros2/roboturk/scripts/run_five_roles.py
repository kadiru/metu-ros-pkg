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


def endpoint(value):
    try:
        host, port = value.rsplit(":", 1)
        if host and 0 < int(port) < 65536:
            return host, int(port)
    except ValueError:
        pass
    raise argparse.ArgumentTypeError("Use HOST:PORT")


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
    p.add_argument("--ssl-vision", type=endpoint, help="UDP destination for SSL-Vision packets")
    p.add_argument("--autoref-ci", type=endpoint, help="TIGERs lockstep vision input")
    p.add_argument("--tracker-output", type=endpoint, help="Forward TIGERs filtered tracker UDP")
    p.add_argument(
        "--stop-after-goal-restart",
        action="store_true",
        help="End after one goal and the responding team puts the ball back in play",
    )
    p.add_argument(
        "--stop-after-placement-reception",
        action="store_true",
        help="End after the receiver controls the official touchline free kick",
    )
    audit = p.add_mutually_exclusive_group()
    audit.add_argument(
        "--capture-exit", type=int, help="Branch for one second at this 1-based local exit"
    )
    audit.add_argument(
        "--exit-tails", type=Path, help="Replay measured post-crossing camera frames"
    )
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--watch", action="store_true")
    mode.add_argument("--record", action="store_true")
    p.add_argument("--placement-batch", type=Path, help="Native placement experiment specification")
    p.add_argument("--open-play-policy", type=Path, help="Trained team-local steering checkpoint")
    p.add_argument(
        "--steer-from-tick",
        type=int,
        default=0,
        help="Replay the released policy before this control tick",
    )
    p.add_argument(
        "--motion-audit",
        action="store_true",
        help="Record measured ball heights and crossing states",
    )
    mode.add_argument(
        "--frame-images", action="store_true", help="Save native JPEG samples at 12.5 Hz"
    )
    a = p.parse_args()
    if a.steer_from_tick < 0 or (a.steer_from_tick and not a.open_play_policy):
        p.error("A delayed steering switch needs a policy and nonnegative tick")
    if a.seconds < 1:
        p.error("Use a positive duration")
    if a.tracker_output and not a.autoref_ci:
        p.error("--tracker-output requires --autoref-ci")
    if a.stop_after_placement_reception and a.referee != "ssl":
        p.error("--stop-after-placement-reception requires the official referee")
    if a.stop_after_goal_restart and a.referee != "ssl":
        p.error("--stop-after-goal-restart requires the official referee")
    if a.capture_exit is not None and (a.referee != "local" or a.capture_exit < 1):
        p.error("--capture-exit requires local play and a positive exit number")
    if a.exit_tails and (a.referee != "local" or not (a.autoref_ci or a.ssl_vision)):
        p.error("--exit-tails requires a local match with vision output")
    dest = a.output.resolve()
    dest.mkdir(parents=True, exist_ok=False)
    (dest / "config.json").write_text(
        json.dumps(
            dict(
                root=str(ROOT),
                seed=a.seed,
                seconds=a.seconds,
                record=a.record,
                frame_images=a.frame_images,
                motion_audit=a.motion_audit,
                open_play_policy=json.loads(a.open_play_policy.read_text())
                if a.open_play_policy
                else None,
                steer_from_tick=a.steer_from_tick,
                placement_batch=json.loads(a.placement_batch.read_text())
                if a.placement_batch
                else None,
                referee=a.referee,
                stop_after_goal_restart=a.stop_after_goal_restart,
                stop_after_placement_reception=a.stop_after_placement_reception,
                capture_exit=a.capture_exit,
                exit_tails=str(a.exit_tails.resolve()) if a.exit_tails else None,
                vision=a.ssl_vision,
                autoref_ci=a.autoref_ci,
                tracker=a.tracker_output,
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
        "--mode=realtime"
        if a.watch or (a.referee == "ssl" and not a.placement_batch)
        else "--mode=fast",
    ]
    if not (a.watch or a.record or a.frame_images):
        cmd.extend(["--no-rendering", "--minimize"])
    with (dest / "webots.log").open("w") as log:
        subprocess.run(
            [*cmd, str(WORLD)], env=env, stdout=log, stderr=subprocess.STDOUT, check=True
        )
    if a.placement_batch:
        if not (dest / "placement.json").is_file():
            raise RuntimeError(f"Missing placement report. See {dest}/webots.log")
        return
    if a.capture_exit and not (dest / "tail.pb").exists():
        raise RuntimeError("Requested exit did not occur")
    if not (dest / "positions.npz").exists():
        raise RuntimeError(f"Missing telemetry. See {dest}/webots.log")


if __name__ == "__main__":
    main()
