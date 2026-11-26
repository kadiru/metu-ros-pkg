"""Audit exclusive roller possession from the original recovery to its first pass."""

import numpy as np
from .pressure_audit import pressure_result


def first_loss(states, game, chain):
    start = chain["recovery"]
    winner = chain["winner"]
    stop = next(
        (
            e["tick"] * 4 + 4
            for e in game["events"]
            if e["event"] not in ("start", "kick") and e["tick"] * 4 + 3 >= start
        ),
        len(states),
    )
    kick = next(
        (e for e in game["events"] if e["event"] == "kick" and start <= e["tick"] * 4 < stop), None
    )
    end = kick["tick"] * 4 if kick else stop
    passes = {p["tick"]: p for p in game["passes"]}
    passed = bool(
        kick
        and (p := passes.get(kick["tick"]))
        and p["team"] == chain["team"]
        and p["passer"] == winner % 3
    )
    s = states[start:end]
    xy = s[:, 4:].reshape(-1, 6, 3)
    d = s[:, None, :2] - xy[:, :, :2]
    c, t = np.cos(xy[:, :, 2]), np.sin(xy[:, :, 2])
    x, y = d[:, :, 0], d[:, :, 1]
    mouths = (abs(c * x + t * y - 0.115) < 0.035) & (abs(c * y - t * x) < 0.045)
    split = np.flatnonzero((mouths.sum(1) != 1) | ~mouths[:, winner])
    sample = start + int(split[0]) if len(split) else None
    return dict(
        prepass_loss=sample is not None or not passed,
        first_break=sample,
        first_pass=kick["tick"] if passed else None,
    )


def touch_result(states, game, protocol):
    row = pressure_result(states, game, protocol)
    row["prepass_loss"] = False
    chain = row.get("chain", {})
    if "winner" in chain:
        row.update(first_loss(states, game, chain))
    if row["qualified"] and row["prepass_loss"]:
        row.update(qualified=False, outcome="interrupted carrier control")
    return row
