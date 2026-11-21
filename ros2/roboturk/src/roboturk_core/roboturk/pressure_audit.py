"""Measure a closing opponent and continuous, body-shielded sole possession at 100 Hz."""

import numpy as np

from .lead_audit import lead_result


def pressure_result(states, game, protocol):
    row = lead_result(states, game, protocol)
    row["lead_goal"] = row["qualified"]
    row["qualified"] = False
    if "contact" not in row:
        return row
    a = row["contact"]
    side = row["chain"]["team"] == "yellow"
    k = 3 * side + row["receiver"]
    robots = states[:, 4:].reshape(-1, 6, 3)
    rivals = np.arange(3) + 3 * (1 - side)
    nearest = int(
        rivals[np.argmin(np.linalg.norm(robots[a, rivals, :2] - robots[a, k, :2], axis=1))]
    )
    toward = robots[:, k, :2] - robots[:, nearest, :2]
    dist = np.linalg.norm(toward, axis=1)
    velocity = (robots[a, nearest, :2] - robots[a - 12, nearest, :2]) / 0.12
    approach = float(velocity @ toward[a] / max(dist[a], 1e-8))
    pressure = bool(dist[a] <= protocol["pressure_m"] and approach >= protocol["approach_mps"])
    row.update(marker=nearest % 3, pressure=pressure, gap_m=float(dist[a]), approach_mps=approach)
    end = min((e["tick"] * 4 for e in game["events"] if e["tick"] * 4 > a), default=len(states))
    delta = states[a:end, None, :2] - robots[a:end, :, :2]
    yaw = robots[a:end, :, 2]
    c, s = np.cos(yaw), np.sin(yaw)
    x, y = delta[:, :, 0], delta[:, :, 1]
    held = (abs(x * c + y * s - 0.115) < 0.035) & (abs(y * c - x * s) < 0.045)
    broken = np.flatnonzero((held.sum(axis=1) != 1) | ~held[:, k])
    n = int(broken[0]) if len(broken) else len(held)
    mouth = delta[:n, k]
    body = toward[a : a + n]
    # Ball and defender lie on opposite sides of the receiver.
    alignment = (mouth * body).sum(1) / np.maximum(
        1e-8, np.linalg.norm(mouth, axis=1) * dist[a : a + n]
    )
    protected = (alignment >= protocol["shield_cosine"]) & (dist[a : a + n] <= protocol["shield_m"])
    duration = protocol["shield_samples"]
    runs = (
        np.flatnonzero(
            np.convolve(protected.astype(int), np.ones(duration, int), "valid") == duration
        )
        if n >= duration
        else []
    )
    row.update(sole_samples=n, shield=False)
    if len(runs):
        row.update(
            shield=True, shield_start=a + int(runs[0]), shield_end=a + int(runs[0]) + duration - 1
        )
    if row["lead_goal"]:
        row["qualified"] = bool(
            pressure and row["shield"] and row["chain"]["scorer"] == row["receiver"]
        )
        row["outcome"] = (
            "pressured shield goal"
            if row["qualified"]
            else "goal without closing pressure"
            if not pressure
            else "goal without receiver shield"
        )
    return row
