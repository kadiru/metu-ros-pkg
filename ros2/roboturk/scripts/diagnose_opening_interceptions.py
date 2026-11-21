"""Locate the first unintended mouth contact of each intercepted opening pass."""

from pathlib import Path
import argparse
import numpy as np
from run_recovery_attack import read, save
from roboturk.blocked_outlet import corridor


def diagnose(folder):
    rows = []
    for path in sorted(folder.glob("game-*.json")):
        game = read(path)
        r = game["opening"]
        if r["outcome"] != "intercepted":
            continue
        states = np.load(path.with_name(path.name.replace("game", "trace").replace("json", "npz")))[
            "states"
        ]
        kick = 4 * r["kick"]
        before = states[kick - 1]
        ball = before[:2]
        pos = before[4:].reshape(6, 3)
        offset = 3 * int(r["chain"]["team"] == "yellow")
        receiver = offset + r["receiver"]
        passer = offset + r["passer"]
        span = states[kick : 4 * r["end"] + 4]
        robots = span[:, 4:].reshape(-1, 6, 3)
        delta = span[:, None, :2] - robots[:, :, :2]
        co, si = np.cos(robots[:, :, 2]), np.sin(robots[:, :, 2])
        held = (abs(co * delta[:, :, 0] + si * delta[:, :, 1] - 0.115) < 0.035) & (
            abs(-si * delta[:, :, 0] + co * delta[:, :, 1]) < 0.045
        )
        held[:, [passer, receiver]] = False
        sample, robot = np.argwhere(held)[0]
        stop = span[sample, :2]
        release_lane = corridor(*ball, *pos[receiver, :2], pos[3 - offset : 6 - offset, :2])
        rows.append(
            dict(
                seed=game["seed"],
                team=r["chain"]["team"],
                interceptor=int(robot),
                opponent=bool(robot // 3 != offset // 3),
                delay_s=round(float((sample + 1) * 0.01), 3),
                ball_travel_m=round(float(np.linalg.norm(stop - ball)), 3),
                planned_length_m=round(float(np.linalg.norm(pos[receiver, :2] - ball)), 3),
                interceptor_distance_at_release_m=round(
                    float(np.linalg.norm(pos[robot, :2] - ball)), 3
                ),
                receiver_distance_at_contact_m=round(
                    float(np.linalg.norm(robots[sample, receiver, :2] - stop)), 3
                ),
                receiver_movement_m=round(
                    float(np.linalg.norm(robots[sample, receiver, :2] - pos[receiver, :2])), 3
                ),
                decision_clearance_m=round(r["lane"], 3),
                release_clearance_m=round(float(release_lane), 3),
                release_xy=ball.tolist(),
                contact_xy=stop.tolist(),
            )
        )
    return rows


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("panel", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    save(a.output, diagnose(a.panel))
