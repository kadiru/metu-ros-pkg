"""Find the first event that breaks an opening recovery using the 100 Hz ball and mouth traces."""

import argparse
from collections import Counter
from pathlib import Path
import numpy as np
from run_recovery_attack import read, save


def classify(game, states):
    e = (game["conversions"] or [{}])[0]
    row = dict(seed=game["seed"])
    if "recovery" not in e:
        return dict(**row, outcome="no exclusive recovery")
    start, winner = e["recovery"], e["winner"]
    row.update(team=e["team"], recovery_s=start / 100, origin=states[start, :2].tolist())
    if "goal" in e:
        return dict(**row, outcome="goal")
    robots = states[:, 4:].reshape(-1, 6, 3)
    delta = states[:, None, :2] - robots[:, :, :2]
    c, s = np.cos(robots[:, :, 2]), np.sin(robots[:, :, 2])
    mouths = (abs(c * delta[:, :, 0] + s * delta[:, :, 1] - 0.115) < 0.035) & (
        abs(c * delta[:, :, 1] - s * delta[:, :, 0]) < 0.045
    )
    stop = next(
        (
            v
            for v in game["events"]
            if v["event"] not in ("start", "kick") and 4 * v["tick"] + 3 >= start
        ),
        None,
    )
    end = min(len(states) - 1, start + 1800, 4 * stop["tick"] + 3 if stop else len(states) - 1)
    rival = mouths[:, 3:] if winner < 3 else mouths[:, :3]
    touch = np.flatnonzero(rival[start : end + 1].any(axis=1))
    terminal = start + int(touch[0]) if len(touch) else end
    kicks = [
        v
        for v in game["events"]
        if v["event"] == "kick" and v["team"] == e["team"] and start <= 4 * v["tick"] <= terminal
    ]
    opts = [
        o["action"] for o in game["options"] if o["recovery"] == start and 4 * o["tick"] <= terminal
    ]
    stage = (
        "before first kick"
        if not kicks
        else ("after pass" if kicks[-1]["speed"] == 2.2 else "after shot")
    )
    outcome = (
        "opponent touch"
        if len(touch)
        else stop["event"]
        if stop and end == 4 * stop["tick"] + 3
        else "time limit"
    )
    return dict(
        **row,
        outcome=outcome + " " + stage,
        elapsed_s=(terminal - start) * 0.01,
        ball=states[terminal, :2].tolist(),
        options=opts,
    )


def analyze(folder):
    games = []
    for f in sorted(folder.glob("game-*.json")):
        with np.load(folder / f.name.replace("game", "trace").replace(".json", ".npz")) as t:
            games.append(classify(read(f), t["states"]))
    return dict(counts=dict(Counter(g["outcome"] for g in games)), games=games)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("folder", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    report = analyze(a.folder)
    save(a.output, report)
    print(report["counts"])
