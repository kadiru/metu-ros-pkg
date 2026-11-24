"""Run or watch one seeded, native 5v5 match and save its role trace."""

import argparse
import gzip
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
    batch = p.add_mutually_exclusive_group()
    batch.add_argument(
        "--replay-kickoffs",
        type=Path,
        help="Compare both planners from recorded restart states in native Webots",
    )
    batch.add_argument(
        "--placement-batch", type=Path, help="Native placement experiment specification"
    )
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
        help="Record robot targets and clearance, plus ball height and crossings",
    )
    mode.add_argument(
        "--frame-images", action="store_true", help="Save native JPEG samples at 12.5 Hz"
    )
    p.add_argument(
        "--frame-window",
        type=float,
        nargs=2,
        metavar=("START", "END"),
        help="Limit native match JPEG capture to these elapsed seconds",
    )
    p.add_argument(
        "--legal-off-ball-from-tick",
        type=int,
        default=0,
        help="Delay off-ball target projection to replay a matched branch",
    )
    p.add_argument(
        "--await-force-start",
        action="store_true",
        help="For paired live fixtures, hold the simulation clock until the first FORCE_START packet",
    )
    p.add_argument(
        "--until-post-game", action="store_true", help="Finish on the official final whistle"
    )
    p.add_argument(
        "--drive-kickoffs",
        action="store_true",
        help="Drive the ten robots to their awarded kickoff positions",
    )
    p.add_argument(
        "--track-kickoffs",
        action="store_true",
        help="Save filtered TIGERs robot positions alongside native frames",
    )
    planners = p.add_mutually_exclusive_group()
    planners.add_argument(
        "--team-kickoffs",
        action="store_true",
        help="Use a separate camera-based kickoff planner for each side",
    )
    planners.add_argument(
        "--parallel-kickoffs",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Use the shared planner with concurrent route reservations",
    )
    p.add_argument(
        "--trajectory-planner",
        action="store_true",
        help="Search timed bang-bang trajectories for off-ball field players in open play",
    )
    p.add_argument("--touchline-recovery", action=argparse.BooleanOptionalAction, default=True)
    batch.add_argument(
        "--touchline-trials", type=int, help="Run this many consecutive 5v5 touchline openings"
    )
    p.add_argument(
        "--errt-fallback",
        action="store_true",
        help="Enable cache-biased RRT after all timed detours fail",
    )
    p.add_argument(
        "--commit-waypoints",
        nargs="?",
        const="on-conflict",
        choices=("on-conflict", "immediate"),
        help="Hold a safe ERRT waypoint after a route switch makes little progress, or immediately",
    )
    p.add_argument(
        "--release-waypoint-early",
        action="store_true",
        help="Release an ERRT waypoint after a faster direct route stays clear",
    )
    batch.add_argument(
        "--crowded-trials",
        type=int,
        help="Run consecutive 5v5 outlet plays against a compact marking block",
    )
    p.add_argument(
        "--film-seed",
        type=int,
        help="Record one seed within a consecutive crowded batch, including its prior reset history",
    )
    p.add_argument("--tackle-policy", type=Path, help="Fitted ball-side tackle checkpoint")
    p.add_argument("--tackle-sweep", type=Path, help="Named tackle profiles for native fitting")
    batch.add_argument(
        "--tackle-trials", type=int, help="Run isolated carrier challenges on the 5v5 field"
    )
    a = p.parse_args()
    if a.tackle_trials is not None and (
        a.tackle_trials < 1
        or a.referee != "local"
        or not a.open_play_policy
        or ((a.frame_images or a.record) and a.tackle_trials != 1)
    ):
        p.error(
            "Tackle trials require a positive count, local referee and open-play policy. Record one start at a time."
        )
    if a.tackle_trials and a.record:
        a.frame_images = True
    if a.tackle_sweep and (a.tackle_trials is None or a.tackle_policy):
        p.error("--tackle-sweep requires --tackle-trials and supplies its own policies")
    tackle_sweep = json.loads(a.tackle_sweep.read_text()) if a.tackle_sweep else None
    if a.tackle_sweep and (
        not isinstance(tackle_sweep, dict)
        or not tackle_sweep
        or any(Path(name).name != name or name in ("", ".", "..") for name in tackle_sweep)
    ):
        p.error("--tackle-sweep must map simple directory names to profiles")
    if a.tackle_policy and not a.open_play_policy:
        p.error("--tackle-policy requires --open-play-policy")
    if a.legal_off_ball_from_tick < 0 or (
        a.legal_off_ball_from_tick
        and (
            a.crowded_trials
            or a.touchline_trials
            or a.tackle_trials
            or a.replay_kickoffs
            or a.placement_batch
        )
    ):
        p.error("--legal-off-ball-from-tick requires a nonnegative tick in a normal match")
    if a.frame_window and (
        not a.frame_images
        or a.crowded_trials
        or a.touchline_trials
        or a.tackle_trials
        or a.replay_kickoffs
        or a.placement_batch
        or not 0 <= a.frame_window[0] < a.frame_window[1] <= a.seconds
    ):
        p.error(
            "--frame-window requires --frame-images and an increasing interval within the match"
        )
    if a.film_seed is not None:
        if not a.crowded_trials or not a.seed <= a.film_seed < a.seed + a.crowded_trials:
            p.error("--film-seed must select a seed within --crowded-trials")
        a.frame_images = True
    if a.release_waypoint_early and not a.commit_waypoints:
        p.error("--release-waypoint-early requires --commit-waypoints")
    if a.commit_waypoints and not a.errt_fallback:
        p.error("--commit-waypoints requires --errt-fallback")
    if a.errt_fallback and not a.trajectory_planner:
        p.error("--errt-fallback requires --trajectory-planner")
    if a.crowded_trials is not None and (
        a.crowded_trials < 1 or a.referee != "local" or not a.trajectory_planner
    ):
        p.error("Crowded trials require a positive count, local referee and trajectory planner")
    if a.crowded_trials and (a.frame_images or a.record):
        if a.crowded_trials != 1 and a.film_seed is None:
            p.error("Select one crowded trial with --film-seed")
        a.frame_images = True
    if a.trajectory_planner and not a.open_play_policy:
        p.error("--trajectory-planner requires --open-play-policy")
    if a.touchline_trials is not None and (
        a.touchline_trials < 1 or a.referee != "local" or not a.open_play_policy
    ):
        p.error(
            "Touchline trials need a positive count, local referee and open-play steering policy"
        )
    if a.touchline_trials and (a.frame_images or a.record):
        if a.touchline_trials != 1:
            p.error("Record a single touchline trial per directory")
        a.frame_images = True
    if a.replay_kickoffs and (a.referee != "local" or a.until_post_game):
        p.error("--replay-kickoffs runs a native replay without a live referee")
    if a.parallel_kickoffs is None:
        a.team_kickoffs = a.team_kickoffs or a.drive_kickoffs
        a.parallel_kickoffs = False
    if a.team_kickoffs and not (a.drive_kickoffs or a.replay_kickoffs):
        p.error("--team-kickoffs requires a driven match or kickoff replays")
    if a.parallel_kickoffs and not a.drive_kickoffs:
        p.error("--parallel-kickoffs requires --drive-kickoffs")
    if a.drive_kickoffs and not a.until_post_game:
        p.error("--drive-kickoffs requires the two-half official fixture")
    if a.track_kickoffs and not a.autoref_ci:
        p.error("--track-kickoffs requires a TIGERs CI connection")
    if a.until_post_game and a.referee != "ssl":
        p.error("--until-post-game requires the official referee")
    if a.until_post_game and any(
        (
            a.await_force_start,
            a.stop_after_goal_restart,
            a.stop_after_placement_reception,
            a.placement_batch,
        )
    ):
        p.error("--until-post-game runs from the official kickoff to the final whistle")
    if a.await_force_start and a.referee != "ssl":
        p.error("--await-force-start requires the official referee")
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
                until_post_game=a.until_post_game,
                drive_kickoffs=a.drive_kickoffs,
                parallel_kickoffs=a.parallel_kickoffs,
                team_kickoffs=a.team_kickoffs,
                track_kickoffs=a.track_kickoffs,
                seconds=a.seconds,
                record=a.record,
                crowded_trials=a.crowded_trials,
                tackle_trials=a.tackle_trials,
                tackle_policy=json.loads(a.tackle_policy.read_text()) if a.tackle_policy else None,
                tackle_sweep=tackle_sweep,
                film_seed=a.film_seed,
                errt_fallback=a.errt_fallback,
                release_waypoint_early=a.release_waypoint_early,
                commit_waypoints=a.commit_waypoints,
                touchline_trials=a.touchline_trials,
                touchline_recovery=a.touchline_recovery,
                trajectory_planner=a.trajectory_planner,
                frame_images=a.frame_images,
                frame_window=a.frame_window,
                legal_off_ball_from_tick=a.legal_off_ball_from_tick,
                motion_audit=a.motion_audit,
                await_force_start=a.await_force_start,
                open_play_policy=json.loads(a.open_play_policy.read_text())
                if a.open_play_policy
                else None,
                steer_from_tick=a.steer_from_tick,
                kickoff_replays=json.loads(a.replay_kickoffs.read_text())
                if a.replay_kickoffs
                else None,
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
    if a.tackle_trials:
        names = tackle_sweep or {"tackle" if a.tackle_policy else "current": None}
        for name in names:
            with gzip.open(dest / name / "trials.jsonl.gz", "rt") as logs:
                if sum(1 for _ in logs) != a.tackle_trials:
                    raise RuntimeError(f"Incomplete tackle trials. See {dest}/webots.log")
        return
    if a.crowded_trials:
        report = dest / "crowded.json"
        if not report.is_file() or len(json.loads(report.read_text())) != a.crowded_trials:
            raise RuntimeError(f"Incomplete crowded trials. See {dest}/webots.log")
        return
    if a.touchline_trials:
        report = dest / "touchline.json"
        if not report.is_file() or len(json.loads(report.read_text())) != a.touchline_trials:
            raise RuntimeError(f"Missing touchline report. See {dest}/webots.log")
        return
    if a.replay_kickoffs:
        if not (dest / "kickoff-replays.json").is_file():
            raise RuntimeError(f"Missing kickoff replay report. See {dest}/webots.log")
        return
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
