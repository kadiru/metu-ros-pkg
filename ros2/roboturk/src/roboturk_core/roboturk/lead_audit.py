"""Reconstruct running receptions and target accuracy independently from commands."""

import numpy as np
from .blocked_audit import opening_result


def lead_result(states, game, protocol):
    row = opening_result(states, game, protocol)
    row["pass_goal"] = row["qualified"]
    row["intercepted"] = row.get("pass_result") == "intercepted"
    if "kick" not in row:
        return row
    plan = game.get("plans", {}).get(str(row["kick"]))
    if not plan:
        row["qualified"] = False
        return row
    a = row["kick"] * 4
    idx = (3 if row["chain"]["team"] == "yellow" else 0) + row["receiver"]
    robots = states[:, 4:].reshape(-1, 6, 3)[:, idx]
    release = robots[a - 1]
    velocity = (release[:2] - robots[a - 5, :2]) / 0.04
    speed = float(np.linalg.norm(velocity))
    heading = velocity / max(1e-8, speed)
    mouth = release[:2] + 0.115 * np.array([np.cos(release[2]), np.sin(release[2])])
    target = np.array(plan["target"])
    lead = float((target - mouth) @ heading)
    row.update(target=target.tolist(), run_speed=speed, lead=lead)
    if row.get("pass_result") != "complete":
        return row
    end = 4 * (row["end"] + 1)
    dr = robots[a:end]
    delta = states[a:end, :2] - dr[:, :2]
    c, s = np.cos(dr[:, 2]), np.sin(dr[:, 2])
    held = (abs(c * delta[:, 0] + s * delta[:, 1] - 0.115) < 0.035) & (
        abs(-s * delta[:, 0] + c * delta[:, 1]) < 0.045
    )
    held &= np.linalg.norm(states[a:end, :2] - states[a - 1, :2], axis=1) >= 0.55
    streak = (
        np.flatnonzero(np.convolve(held.astype(int), np.ones(20, int), "valid") == 20)
        if len(held) >= 20
        else []
    )
    if not len(streak):
        row.update(qualified=False, outcome="unverified reception")
        return row
    contact = a + int(streak[0])
    advance = float((robots[contact, :2] - release[:2]) @ heading)
    error = float(np.linalg.norm(states[contact, :2] - target))
    row.update(contact=contact, advance=advance, target_error=error)
    verified = (
        speed >= protocol["run_speed_mps"]
        and lead >= protocol["lead_distance_m"]
        and advance >= protocol["run_progress_m"]
        and error <= protocol["target_tolerance_m"]
        and plan["runner"] == row["receiver"]
    )
    row["verified_lead"] = bool(verified)
    if row["pass_goal"] and not verified:
        row.update(qualified=False, outcome="goal without verified lead")
    return row
