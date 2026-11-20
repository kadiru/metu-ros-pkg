#!/usr/bin/env python3
"""Label every neural skill on recorded native states; no new physics transitions.

The existing teacher supplies behavior-selection labels. Additional counterfactual
motor labels teach meaningful skill execution even when exploration chooses a
different behavior. This script is never imported by deployed policies.
"""

import argparse
from pathlib import Path
import json
import math
import numpy as np
from roboturk.capture import CaptureTeacher
from roboturk.continuous import encode, ordered
from roboturk.continuous_teacher import ContinuousTeacher, avoid
from roboturk.continuous_training import digest
from roboturk.model import Command, Intent, Skill, body_vector, possession, wrap
from roboturk.self_play_training import write
from relabel_continuous import reconstruct


def all_heads(match, key):
    teacher = ContinuousTeacher()
    chosen = teacher.action(match, key)
    selected = int(chosen[:7].argmax())
    w, m, cfg = match.world, match.memory, match.cfg
    r, b = w.robot(key), w.ball
    keys = ordered(w, key)
    mate = w.robot(keys[1])
    sign = w.direction(key.team)
    held = possession(w, key, cfg)
    chase = CaptureTeacher().command(w, key, Intent(Skill.CHASE, (b.x, b.y)), cfg)
    result = chosen.copy()
    for mode in range(7):
        if mode == selected:
            continue  # Preserve the original demonstrated command exactly.
        if mode == 5:
            gx = -sign * 4.1
            c = teacher.goto(w, key, (gx + 0.38 * (b.x - gx), 0.38 * b.y), (b.x, b.y))
        elif mode == 6:
            target = (
                float(np.clip(b.x + sign * 1.15, -3.4, 3.4)),
                float(np.clip(b.y + (-1 if b.y >= 0 else 1) * 1.35, -2.25, 2.25)),
            )
            c = teacher.goto(w, key, target, (b.x, b.y))
        elif not held:
            c = chase
        else:
            if mode == 2:
                tx, ty, speed = mate.x, mate.y, 0
            elif mode == 3:
                tx, ty, speed = sign * 4.5, (0.25 if b.y >= 0 else -0.25), 0
            else:
                tx, ty, speed = sign * 3.2, 0, 0.6
                closest = min(
                    (w.robot(k) for k in keys[2:]), key=lambda p: math.hypot(p.x - r.x, p.y - r.y)
                )
                if 0 < sign * (closest.x - r.x) < 1.5 and abs(closest.y - r.y) < 0.6:
                    ty = float(
                        np.clip(closest.y + (1 if r.y >= closest.y else -1) * 0.85, -2.1, 2.1)
                    )
                    tx = sign * min(3.1, sign * closest.x + 0.7)
            angle = wrap(math.atan2((ty - b.y) * (0.8 if mode == 3 else 1), tx - b.x) - r.yaw)
            omega = float(np.clip(2.2 * angle, -1.1, 1.1))
            bx, by = body_vector(b.x - r.x, b.y - r.y, r.yaw)
            c = Command(
                speed * max(0, math.cos(angle)) ** 3 + 3 * (bx - cfg.possession_x),
                -cfg.possession_x * omega + 3 * by,
                omega,
                1,
            )
            contested = any(possession(w, k, cfg) for k in keys[2:])
            if mode in (2, 3) and abs(angle) < 0.035 and (m.dwell[key] >= 20 or contested):
                c = Command(kick=2.2 if mode == 2 else 3.5)
        result[7 + 4 * mode : 11 + 4 * mode] = encode(mode, avoid(w, key, c))[
            7 + 4 * mode : 11 + 4 * mode
        ]
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        p.error("Preserve previous labels: use a new output")
    receipt = json.loads((args.source / "collection.json").read_text())
    assert all(4200000 <= m["seed"] < 4220000 for m in receipt["matches"])
    assert digest(args.source / "data.npz") == receipt["data_sha256"]
    data = np.load(args.source / "data.npz")
    x = data["observations"]
    assert x.shape[1] == 122 and len(x) % 4 == 0
    y = []
    for i, row in enumerate(x):
        match, key = reconstruct(row[:88], i % 2)
        y.append(all_heads(match, key))
        if (i + 1) % 20000 == 0:
            print("All-head labels", args.source.name, i + 1, len(x), flush=True)
    args.output.mkdir(parents=True)
    np.savez_compressed(args.output / "data.npz", observations=x, actions=np.asarray(y, np.float32))
    receipt.update(
        source=str(args.source),
        source_data_sha256=receipt["data_sha256"],
        data_sha256=digest(args.output / "data.npz"),
        motor_labels="all_seven_counterfactual_skills",
        label_control="training-only counterfactual commands on recorded native states; no new transitions",
        allheads_script_sha256=digest(Path(__file__)),
    )
    write(args.output / "collection.json", receipt)


if __name__ == "__main__":
    main()
