#!/usr/bin/env python3
"""Audit native six-robot traces and qualify intended recipient choices."""

import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np


def inspect_game(folder, i, result, weights):
    states = np.load(folder / f"trace-{i:02}.npz")["states"]
    frames_per_tick = len(states) // round(result["seconds"] / 0.04)
    assert frames_per_tick == 4
    dt = 0.04 / frames_per_tick
    ball = states[:, :2]
    pose = states[:, 4:].reshape(-1, 6, 3)
    delta = ball[:, None, :] - pose[:, :, :2]
    c, s = np.cos(pose[:, :, 2]), np.sin(pose[:, :, 2])
    bx = c * delta[:, :, 0] + s * delta[:, :, 1]
    by = -s * delta[:, :, 0] + c * delta[:, :, 1]
    mouth = (np.abs(bx - 0.115) < 0.035) & (np.abs(by) < 0.045)
    near = np.zeros(len(states), dtype=bool)
    for a in range(6):
        for b in range(a):
            near |= np.linalg.norm(pose[:, a, :2] - pose[:, b, :2], axis=1) < 0.18
    exits = ((np.abs(pose[:, :, 0]) > 4.41) | (np.abs(pose[:, :, 1]) > 2.91)).any(axis=1)
    assert len(states) == result["stats"]["physical_samples"]
    assert int(near.sum()) == result["stats"]["contacts"]
    assert int(exits.sum()) == result["stats"]["exits"]
    completions = []
    kicks = [
        (e["tick"], e["team"], e["robot"])
        for e in result["events"]
        if e["event"] == "kick" and e["speed"] == 2.2
    ]
    assert len(kicks) == len(result["passes"])
    for p, kick in zip(result["passes"], kicks, strict=True):
        assert kick == (p["tick"], p["team"], p["passer"])
        features = np.asarray(p["features"])
        expected = int(features[1] @ weights[p["team"]] > features[0] @ weights[p["team"]])
        assert p["slot"] == expected
        mates = [j for j in range(3) if j != p["passer"]]
        assert p["receiver"] == mates[expected]
        if p["result"] != "complete":
            continue
        lo, hi = p["tick"] * frames_per_tick, (p["end"] + 1) * frames_per_tick
        assert (p["end"] - p["tick"]) * 0.04 <= 4
        actor = (3 if p["team"] == "yellow" else 0) + p["receiver"]
        holding = mouth[lo:hi, actor] & (mouth[lo:hi].sum(axis=1) == 1)
        travel = np.linalg.norm(ball[lo:hi] - p["ball"], axis=1)
        stable = holding & (travel >= 0.55)
        samples = round(0.20 / dt)
        windows = np.convolve(stable.astype(int), np.ones(samples, int), mode="valid")
        assert len(stable) >= samples and np.any(windows == samples), (result["seed"], p)
        end = np.flatnonzero(windows == samples)[0] + samples
        opponents = [j for j in range(6) if j not in (actor, actor - p["receiver"] + p["passer"])]
        assert not mouth[lo : lo + end, opponents].any()
        completions.append(p)
    return completions


def audit(folder, models, protocol, mode):
    launches = json.loads((folder / "launch.json").read_text())
    assert launches["mode"] == mode
    plan = protocol["qualification" if mode == "qualify" else mode]
    assert launches["matches"] == plan["matches"] and launches["seconds"] == plan.get(
        "seconds", protocol["seconds"]
    )
    total, slots, receivers, pairings, scores = (
        Counter(),
        Counter(),
        Counter(),
        Counter(),
        Counter(),
    )
    games = []
    for i in range(plan["matches"]):
        g = json.loads((folder / f"game-{i:02}.json").read_text())
        assert g["seed"] == plan["first"] + i and g["seconds"] == plan.get(
            "seconds", protocol["seconds"]
        )
        blue, yellow = [(901, 901), (902, 902), (901, 902), (902, 901)][
            i // 8 if mode == "qualify" else 2
        ]
        complete = inspect_game(
            folder, i, g, {"blue": models[str(blue)], "yellow": models[str(yellow)]}
        )
        total.update(g["stats"])
        scores.update(g["score"])
        total.update(
            matches=1,
            passes=len(g["passes"]),
            complete=len(complete),
            with_completion=int(bool(complete)),
        )
        for p in complete:
            slots[f"{p['team']}/{p['slot']}"] += 1
            receivers[f"{p['team']}/{p['receiver']}"] += 1
            pairings[f"{p['team']}/{p['passer']}->{p['receiver']}"] += 1
        games.append(
            dict(seed=g["seed"], passes=len(g["passes"]), complete=len(complete), goals=g["score"])
        )
    rate = total["complete"] / max(1, total["passes"])
    t = protocol["gates"]
    checks = {}
    if mode == "qualify":
        checks = dict(
            finished=total["matches"] == t["finished"],
            with_completion=total["with_completion"] >= t["min_matches_with_intended_completion"],
            completion_rate=rate >= t["min_intended_completions_per_kicked_pass"],
            choices=all(
                slots[f"{team}/{k}"] >= t["min_completed_passes_per_receiver_slot_per_team"]
                for team in ("blue", "yellow")
                for k in range(2)
            ),
            identities=all(
                sum(receivers[f"{team}/{k}"] > 0 for k in range(3))
                >= t["min_distinct_receiver_ids_per_color"]
                for team in ("blue", "yellow")
            ),
            exits=total["exits"] <= t["max_robot_field_exits"],
            contact_fraction=total["contacts"] / total["physical_samples"]
            <= t["max_robot_contact_sample_fraction"],
        )
    if mode in ("open", "demo"):
        assert all(
            e["event"] != "shot clock"
            for f in folder.glob("game-*.json")
            for e in json.loads(f.read_text())["events"]
        )
    if mode == "open":
        criteria = plan["gates"]
        checks = dict(
            with_completion=total["with_completion"] >= criteria["min_matches_with_completion"],
            completion_rate=rate >= criteria["min_completion_rate"],
            choices=all(
                slots[f"{t}/{i}"] >= criteria["minimum_completed_each_slot_per_color"]
                for t in ("blue", "yellow")
                for i in range(2)
            ),
            exits=total["exits"] <= criteria["max_robot_field_exits"],
            contact_fraction=total["contacts"] / total["physical_samples"]
            <= criteria["max_robot_contact_sample_fraction"],
        )
    return dict(
        passed=all(checks.values()),
        checks=checks,
        totals=dict(total),
        completion_rate=rate,
        completed_by_slot=dict(slots),
        completed_by_id=dict(receivers),
        pairs=dict(pairings),
        goals=dict(scores),
        games=games,
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("folder", type=Path)
    p.add_argument("--models", required=True, type=Path)
    p.add_argument("--protocol", required=True, type=Path)
    p.add_argument("--mode", choices=["qualify", "demo", "open"], required=True)
    a = p.parse_args()
    summary = audit(
        a.folder, json.loads(a.models.read_text()), json.loads(a.protocol.read_text()), a.mode
    )
    (a.folder / "audit.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({k: v for k, v in summary.items() if k != "games"}))
    return int(not summary["passed"])


if __name__ == "__main__":
    raise SystemExit(main())
