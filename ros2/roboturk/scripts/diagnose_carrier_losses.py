"""Locate the last ball position in the carrier's roller and the first missed sample."""

import argparse
from pathlib import Path
import numpy as np
from train_recovery_touch import read, save


def diagnose(d):
    losses = []
    for f in sorted(d.glob("game-*.json")):
        g = read(f)
        r = g["opening"]
        chain = r["chain"]
        if r.get("first_break") is None:
            continue
        xy = np.load(f.with_name(f.name.replace("game", "trace").replace("json", "npz")))["states"]
        rs = xy[:, 4:].reshape(-1, 6, 3)
        k = chain["winner"]
        j = r["first_break"]
        opts = [
            o
            for o in g["options"]
            if o["team"] == chain["team"] and o["recovery"] == chain["recovery"]
        ]
        tick = opts[0]["tick"] if opts else (chain["recovery"] + 4) // 4
        a = tick * 4 - 1
        delta = xy[j, :2] - rs[j, k, :2]
        yaw = rs[j, k, 2]
        bx = float(np.cos(yaw) * delta[0] + np.sin(yaw) * delta[1])
        by = float(-np.sin(yaw) * delta[0] + np.cos(yaw) * delta[1])
        rivals = rs[a, 3 if k < 3 else 0 : 6 if k < 3 else 3, :2]
        losses.append(
            dict(
                seed=g["seed"],
                team=chain["team"],
                winner=k % 3,
                option=opts[0]["action"] if opts else None,
                decision_tick=tick,
                recovery_sample=chain["recovery"],
                loss_sample=j,
                seconds_from_decision=(j - a) * 0.01,
                ball_forward_m=bx,
                ball_left_m=by,
                edge="lateral"
                if abs(by) >= 0.045
                else "longitudinal"
                if abs(bx - 0.115) >= 0.035
                else "shared",
                rival_gap_at_decision_m=float(
                    np.min(np.linalg.norm(rivals - rs[a, k, :2], axis=1))
                ),
                speed_at_recovery_mps=float(np.linalg.norm(xy[chain["recovery"], 2:4])),
            )
        )
    return losses


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("panel", type=Path)
    parser.add_argument("output", type=Path)
    a = parser.parse_args()
    save(a.output, diagnose(a.panel))
