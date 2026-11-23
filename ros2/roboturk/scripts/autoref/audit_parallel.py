"""Compare concurrent kickoff motion in paired official matches and matched Webots replays."""

import argparse
import json
from pathlib import Path
from statistics import mean, median

import numpy as np

from audit_kickoff import measure
from roboturk.five_roles import CFG, kickoff
from roboturk.model import wrap
from run_fixture import ROOT
from run_league import ACTOR, PARALLEL_ARMS, PARALLEL_SEEDS
from train_open_motion import read_game, save


def motion(states, dt):
    xy = states[:, 4:].reshape(-1, 10, 3)[:, :, :2]
    moving = (np.linalg.norm(np.diff(xy, axis=0), axis=2) > 0.2 * dt).sum(axis=1)
    a, b = np.triu_indices(10, 1)
    gaps = np.linalg.norm(xy[:, a] - xy[:, b], axis=2)
    return dict(
        moving_mean=float(moving.mean()) if len(moving) else 0.0,
        moving_peak=int(moving.max(initial=0)),
        closest_centers_m=float(gaps.min()),
        newly_under_diameter=int(
            np.sum((gaps.min(axis=0) < 2 * CFG.robot_radius) & (gaps[0] >= 2 * CFG.robot_radius))
        ),
    )


def describe(values):
    return dict(
        n=len(values),
        median=round(median(values), 3),
        mean=round(mean(values), 3),
        max=round(max(values), 3),
    )


def matches(root):
    cases, pairs = [], []
    actor = json.loads((ROOT / ACTOR).read_text())
    for seed in PARALLEL_SEEDS:
        pair = {"seed": seed}
        openings = []
        for arm in PARALLEL_ARMS:
            path = root / str(seed) / arm
            spec = json.loads(path.with_suffix(".json").read_text())
            cfg = json.loads((path / "match/config.json").read_text())
            assert cfg["seed"] == spec["seed"] == seed and spec["half_seconds"] == 30
            assert (
                cfg["open_play_policy"] == actor and cfg["drive_kickoffs"] and cfg["track_kickoffs"]
            )
            assert cfg["parallel_kickoffs"] == (arm == "parallel")
            assert cfg["frame_images"] == (seed == PARALLEL_SEEDS[0])
            report, state, velocity = measure(path, True)
            openings.append(np.concatenate((state, np.ravel(velocity))))
            log, states = read_game(path / "match")
            starts = {e["tick"]: e for e in log["events"] if e["event"] == "formation_start"}
            formations = report["formations"]
            for f in formations:
                start, end = f["start"], f["end"]
                first = log["frames"][start]
                f["kind"] = (
                    "opening"
                    if f["restart"] == 1
                    else "half-time"
                    if first["stage"] == "NORMAL_SECOND_HALF_PRE"
                    else "after goal"
                )
                f.update(motion(states[start : end + 1], log["dt"]))
                if arm == "current" and f["restart"] > 1:
                    cases.append(
                        dict(
                            seed=seed,
                            restart=f["restart"],
                            kind=f["kind"],
                            serving=f["command"].split("_")[-1].lower(),
                            sign=first["attack_sign"],
                            keys=log["keys"],
                            ball=states[start, :4].tolist(),
                            poses=states[start, 4:].reshape(10, 3).tolist(),
                            velocities=starts[start]["velocities"],
                        )
                    )
            calls = report["observed_crashes"]
            for call in calls:
                call["during_setup"] = any(
                    f["start"] <= call["tick"] < f["end"] for f in formations
                )
            pair[arm] = dict(
                formations=formations,
                crashes=calls,
                accepted=len(report["accepted_crashes"]),
                goals=report["final_score"],
                passes=len(report["completed"]),
                attempts=report["attempts"],
                placement_failures=report["accepted_events"].get("PLACEMENT_FAILED", 0),
            )
        assert np.array_equal(*openings), f"Different first-action state on {seed}"
        pair["opening_max_difference"] = float(np.abs(openings[0] - openings[1]).max())
        pairs.append(pair)
    totals = {}
    for arm in PARALLEL_ARMS:
        rows = [p[arm] for p in pairs]
        formations = [f for r in rows for f in r["formations"]]
        travel = [f for f in formations if f["restart"] > 1 and f["released"]]
        calls = [c for r in rows for c in r["crashes"]]
        totals[arm] = dict(
            awarded=len(formations),
            released=sum(f["released"] for f in formations),
            setup=describe([f["native_s"] for f in travel]),
            half_time=describe([f["native_s"] for f in travel if f["kind"] == "half-time"]),
            moving_mean=sum(f["moving_mean"] * f["native_s"] for f in travel)
            / sum(f["native_s"] for f in travel),
            moving_peak=max(f["moving_peak"] for f in travel),
            closest_centers_m=min(f["closest_centers_m"] for f in formations),
            newly_under_diameter=sum(f["newly_under_diameter"] for f in formations),
            raw=len(calls),
            accepted=sum(r["accepted"] for r in rows),
            setup_raw=sum(c["during_setup"] for c in calls),
            setup_accepted=sum(bool(c["official_id"]) and c["during_setup"] for c in calls),
            goals=sum(sum(r["goals"]) for r in rows),
            passes=sum(r["passes"] for r in rows),
            attempts=sum(r["attempts"] for r in rows),
        )
    return dict(pairs=pairs, live_totals=totals), cases


def replays(folder, cases):
    manifest = json.loads((folder / "kickoff-replays.json").read_text())
    index = {(r["seed"], r["restart"], r["arm"]): r for r in manifest}
    assert len(index) == len(manifest) == 2 * len(cases)
    rows = []
    with np.load(folder / "kickoff-traces.npz") as traces:
        for case in cases:
            opening = []
            row = {k: case[k] for k in ("seed", "restart", "kind")}
            target = kickoff(case["seed"] + 101 * case["restart"], case["serving"], case["sign"])
            for arm in PARALLEL_ARMS:
                r = index[case["seed"], case["restart"], arm]
                states = traces[r["name"]]
                opening.append(np.concatenate((states[0], np.ravel(r["initial_velocities"]))))
                assert r["ready"] and abs(r["seconds"] - (len(states) - 1) * CFG.dt) < 1e-6
                end = states[-1, 4:].reshape(10, 3)
                errors = end - np.array([(g.x, g.y, g.yaw) for g in target.robots])
                assert np.linalg.norm(errors[:, :2], axis=1).max() <= 0.0041
                assert all(abs(wrap(yaw)) < 0.015 for yaw in errors[:, 2])
                row[arm] = dict(seconds=r["seconds"], **motion(states, CFG.dt))
            np.testing.assert_allclose(*opening, rtol=0, atol=1e-9)
            row["opening_max_difference"] = float(np.abs(opening[0] - opening[1]).max())
            rows.append(row)
    totals = {arm: describe([r[arm]["seconds"] for r in rows]) for arm in PARALLEL_ARMS}
    totals["parallel_faster"] = sum(
        r["parallel"]["seconds"] < r["current"]["seconds"] for r in rows
    )
    return dict(trials=rows, totals=totals)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("fresh", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--write-replay-cases", type=Path)
    p.add_argument("--replays", type=Path)
    args = p.parse_args()
    report, cases = matches(args.fresh)
    if args.write_replay_cases:
        save(args.write_replay_cases, cases)
    if args.replays:
        report["matched_replays"] = replays(args.replays, cases)
    save(args.output, report)
