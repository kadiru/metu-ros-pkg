"""Compare live tracker positions, official calls and physical kickoff paths."""

import argparse
from collections import Counter
from itertools import groupby
import json
from pathlib import Path

import numpy as np

from audit_league import audit
from roboturk.five_roles import kickoff
from roboturk.model import wrap
from run_league import ACTOR, KICKOFF_ARMS, KICKOFF_SEEDS
from run_fixture import ROOT
from train_open_motion import read_game, save


def tracked_positions(log):
    frames = log["frames"]
    tracks = np.full((len(frames), len(log["keys"]), 2), np.nan)
    indexes = {(2 if team == "blue" else 1, num): i for i, (team, num) in enumerate(log["keys"])}
    for i, f in enumerate(frames):
        if "tracker_time" not in f:
            continue
        # In the lockstep transport these are estimates for the same camera timestamp.
        assert abs(f["tracker_time"] - f["vision_time"]) < 0.001
        for team, number, x, y in f["tracked_xy"]:
            k = indexes[team, number]
            assert np.isnan(tracks[i, k]).all() and np.isfinite([x, y]).all()
            tracks[i, k] = x, y
    return tracks


def formation_report(log, states, tracks, driven):
    frames = log["frames"]
    xyz = states[:, 4:].reshape(-1, 10, 3)
    xy = xyz[:, :, :2]
    gaps = np.linalg.norm(tracks - xy, axis=2)
    jumps = np.linalg.norm(np.diff(xy, axis=0, prepend=xy[:1]), axis=2)
    ready = {e["restart"]: e["tick"] for e in log["events"] if e["event"] == "formation_ready"}
    records = []
    for command, group in groupby(enumerate(frames), lambda v: v[1]["ssl_command"]):
        section = list(group)
        if not command.startswith("PREPARE_KICKOFF_"):
            continue
        start, f = section[0]
        stop = section[-1][0] + 1
        n = f["restart"]
        # Include early open play for a short PREPARE so both variants get two seconds.
        window = gaps[start : min(len(frames), start + round(2 / log["dt"]))]
        through_release = gaps[start : min(len(frames), stop + round(2 / log["dt"]))]
        moving = jumps[start:stop] > 0.2 * log["dt"]
        moving_errors = gaps[start:stop][moving]
        target = kickoff(log["seed"] + 101 * n, command.split("_")[-1].lower(), f["attack_sign"])
        target_xy = np.array([(r.x, r.y) for r in target.robots])
        assert [[r.key.team, r.key.id] for r in target.robots] == log["keys"]
        released = stop < len(frames) and frames[stop]["ssl_command"] == "NORMAL_START"
        row = dict(
            restart=n,
            command=command,
            start=start,
            end=stop,
            released=released,
            native_s=round((stop - start) * log["dt"], 2),
            max_step_m=float(jumps[start:stop].max()),
            full_max_track_error_m=float(np.nanmax(through_release))
            if np.isfinite(through_release).any()
            else None,
            full_tracked_samples=int(np.isfinite(through_release).sum()),
            full_expected_samples=int(through_release.size),
            full_robot_samples_error_over_30cm=int((through_release > 0.30).sum()),
            moving_max_track_error_m=float(np.nanmax(moving_errors))
            if np.isfinite(moving_errors).any()
            else None,
            tracked_samples=int(np.isfinite(window).sum()),
            expected_samples=int(window.size),
            max_track_error_2s_m=float(np.nanmax(window)) if np.isfinite(window).any() else None,
            robot_samples_error_over_30cm=int((window > 0.30).sum()),
        )
        if released:
            row.update(
                target_error_m=float(np.linalg.norm(xy[stop] - target_xy, axis=1).max()),
                yaw_error_rad=max(
                    abs(wrap(a - r.yaw))
                    for a, r in zip(xyz[stop, :, 2], target.robots, strict=True)
                ),
                ball_offset_m=float(np.linalg.norm(states[stop, :2])),
            )
            if driven:
                assert start <= ready[n] < stop
                assert row["target_error_m"] < 0.0041 and row["yaw_error_rad"] < 0.016
                assert row["ball_offset_m"] < 0.0041
        records.append(row)
    if driven:
        assert max(jumps.ravel()) < 0.10, "Discontinuous robot motion in physical restart match"
    return records, float(jumps.max())


def measure(folder, driven):
    r, opening, velocities = audit(folder, require_native_goals=False)
    log, states = read_game(folder / "match")
    tracks = tracked_positions(log)
    records, biggest = formation_report(log, states, tracks, driven)
    for c in r["observed_crashes"]:
        ids = [log["keys"].index([team, c[team]]) for team in ("blue", "yellow")]
        tick = c["tick"]
        c["restart"] = log["frames"][tick]["restart"]
        c["stage"] = log["frames"][tick]["stage"]
        window = tracks[max(0, tick - round(1 / log["dt"])) : tick + 1]
        distance = np.linalg.norm(window[:, ids[0]] - window[:, ids[1]], axis=1)
        c["tracker_pair_samples"] = int(np.isfinite(distance).sum())
        c["closest_tracked_centers_m"] = (
            float(np.nanmin(distance)) if c["tracker_pair_samples"] else None
        )
    r.update(formations=records, max_robot_step_m=biggest)
    return r, opening, velocities


def summarize(root):
    pairs = []
    actor = json.loads((ROOT / ACTOR).read_text())
    totals = {arm: Counter() for arm in KICKOFF_ARMS}
    for seed in KICKOFF_SEEDS:
        pair = dict(seed=seed)
        opening = velocities = None
        for arm in KICKOFF_ARMS:
            folder = root / str(seed) / arm
            spec = json.loads(folder.with_suffix(".json").read_text())
            cfg = json.loads((folder / "match/config.json").read_text())
            assert cfg["seed"] == spec["seed"] == seed and spec["half_seconds"] == 30
            assert cfg["open_play_policy"] == actor and cfg["track_kickoffs"]
            assert cfg["drive_kickoffs"] == (arm == "drive")
            assert cfg["frame_images"] == (seed == KICKOFF_SEEDS[0])
            r, s, v = measure(folder, arm == "drive")
            if opening is not None:
                assert np.array_equal(opening, s) and np.array_equal(velocities, v)
            opening, velocities = s, v
            formations = r["formations"]
            calls = r["observed_crashes"]
            totals[arm].update(
                dict(
                    observed=len(calls),
                    accepted=len(r["accepted_crashes"]),
                    accepted_distant=sum(
                        bool(c["official_id"]) and c["nearest_centers_previous_second_m"] > 0.30
                        for c in calls
                    ),
                    observed_distant=sum(
                        c["nearest_centers_previous_second_m"] > 0.30 for c in calls
                    ),
                    after_first_accepted_distant=sum(
                        bool(c["official_id"])
                        and c["restart"] > 1
                        and c["nearest_centers_previous_second_m"] > 0.30
                        for c in calls
                    ),
                    passes=len(r["completed"]),
                    attempts=r["attempts"],
                    goals=len(r["goals"]),
                    full_ball_goals=sum(g["crossing_tick"] is not None for g in r["goals"]),
                    native_play_seconds=sum(h["running_native_s"] for h in r["official_halves"]),
                    kickoffs=len(formations),
                    released=sum(f["released"] for f in formations),
                    measurements=sum(f["tracked_samples"] for f in formations),
                    expected=sum(f["expected_samples"] for f in formations),
                    through_release_samples=sum(f["full_tracked_samples"] for f in formations),
                    through_release_expected=sum(f["full_expected_samples"] for f in formations),
                    through_release_over_30cm=sum(
                        f["full_robot_samples_error_over_30cm"] for f in formations
                    ),
                    off_by_30cm=sum(f["robot_samples_error_over_30cm"] for f in formations),
                )
            )
            pair[arm] = r
        pairs.append(pair)
    for counts in totals.values():
        counts["native_play_seconds"] = round(counts["native_play_seconds"], 2)
    return dict(pairs=pairs, totals=totals)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument(
        "--controls",
        action="store_true",
        help="Verify the pilot and the close-approach control match",
    )
    args = p.parse_args()
    if args.controls:
        report = {name: measure(args.root / name, True)[0] for name in ("pilot", "positive")}
    else:
        report = summarize(args.root)
    save(args.output, report)
