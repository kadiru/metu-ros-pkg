#!/usr/bin/env python3
"""Correct training labels on recorded native states; generate no synthetic transitions."""

import argparse
from pathlib import Path
import json
import math
from types import SimpleNamespace
import numpy as np
from roboturk.model import Ball, Config, Referee, Robot, RobotKey, World
from roboturk.continuous import Memory, observation
from roboturk.continuous_teacher import ContinuousTeacher
from roboturk.continuous_training import digest
from roboturk.self_play_training import write


def reconstruct(x, robot_id):
    keys = (
        RobotKey("blue", robot_id),
        RobotKey("blue", 1 - robot_id),
        RobotKey("yellow", 0),
        RobotKey("yellow", 1),
    )
    robots = []
    for i, k in enumerate(keys):
        p = x[i * 9 : (i + 1) * 9]
        robots.append(
            Robot(
                k,
                float(p[0] * 4.5),
                float(p[1] * 3),
                math.atan2(float(p[5]), float(p[4])),
                float(p[2] * 2),
                float(p[3] * 2),
                float(p[6] * 4),
            )
        )
    time = 45 - float(x[86]) * 45
    w = World(
        time,
        tuple(sorted(robots, key=lambda r: r.key)),
        Ball(float(x[36] * 4.5), float(x[37] * 3), float(x[38] * 4), float(x[39] * 4)),
        Referee.RUNNING,
    )
    m = Memory()
    m.reset(w)
    m.dwell.update({k: round(float(x[i * 9 + 8]) * 15) for i, k in enumerate(keys)})
    m.mode[keys[0]] = int(np.argmax(x[79:86]))
    m.last_kick.update({keys[i]: time - float(x[76 + i]) * 5 for i in range(2)})
    m.last_control = time - float(x[78]) * 5
    m.scores["blue"] = round(float(x[87]) * 5)
    return SimpleNamespace(world=w, memory=m, cfg=Config()), keys[0]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        p.error("Preserve prior labels; output must be new")
    receipt = json.loads((args.source / "collection.json").read_text())
    assert all(4200000 <= m["seed"] < 4220000 for m in receipt["matches"])
    assert digest(args.source / "data.npz") == receipt["data_sha256"]
    data = np.load(args.source / "data.npz")
    assert len(data["observations"]) % 4 == 0
    x, old = data["observations"], data["actions"]
    teacher = ContinuousTeacher()
    xs, ys = [], []
    max_delta, changed_modes = 0.0, 0
    for i, row in enumerate(x):
        # Every collection tick stores blue0,blue1,yellow0,yellow1 in that order.
        match, key = reconstruct(row[:88], i % 2)
        obs = observation(match.world, key, match.memory)
        max_delta = max(max_delta, float(np.max(np.abs(obs[:88] - row[:88]))))
        y = teacher.action(match, key)
        changed_modes += int(np.argmax(y[:7]) != np.argmax(old[i, :7]))
        xs.append(obs)
        ys.append(y)
        if (i + 1) % 20000 == 0:
            print(
                "Relabel",
                args.source.name,
                i + 1,
                len(x),
                "mode changes",
                changed_modes,
                flush=True,
            )
    # Float32 reconstruction can cross a contact predicate exactly at its boundary;
    # count and retain that measurement effect explicitly rather than hiding it.
    args.output.mkdir(parents=True)
    np.savez_compressed(
        args.output / "data.npz", observations=np.asarray(xs), actions=np.asarray(ys)
    )
    root = Path(__file__).resolve().parents[1]
    receipt.update(
        data_sha256=digest(args.output / "data.npz"),
        label_control="offline teacher relabel of recorded native states; no new transitions",
        source=str(args.source),
        source_data_sha256=digest(args.source / "data.npz"),
        label_revision={
            "teacher_sha256": digest(root / "src/roboturk_core/roboturk/continuous_teacher.py"),
            "features_sha256": digest(root / "src/roboturk_core/roboturk/continuous_features.py"),
            "script_sha256": digest(Path(__file__)),
            "changed_modes": changed_modes,
            "max_base_observation_reconstruction_delta": max_delta,
        },
    )
    write(args.output / "collection.json", receipt)
    print(receipt["label_revision"], flush=True)


if __name__ == "__main__":
    main()
