"""Score complete passes and high-speed encounters from native 5v5 telemetry."""

import math
import numpy as np


def stats(log, states, start=0):
    frames, keys = log["frames"], log["keys"]
    assert np.isfinite(states).all() and states.shape == (len(frames), 34)
    xyz = states[:, 4:].reshape(-1, 10, 3)
    d = states[:, :2, None] - xyz[:, :, :2].transpose(0, 2, 1)
    c, s = np.cos(xyz[:, :, 2]), np.sin(xyz[:, :, 2])
    grip = (abs(d[:, 0] * c + d[:, 1] * s - 0.115) < 0.035) & (
        abs(-d[:, 0] * s + d[:, 1] * c) < 0.045
    )
    exclusive = grip & (grip.sum(axis=1)[:, None] == 1)
    events = log["events"]
    kicks = [e for e in events if e["event"] in ("pass", "shot")]
    passes = []
    for index, e in enumerate(kicks):
        k = e["tick"]
        if e["event"] != "pass" or k < start:
            continue
        stop = min(
            len(states),
            k + 51,
            kicks[index + 1]["tick"] + 1 if index + 1 < len(kicks) else len(states),
        )
        for i in range(k + 3, stop):
            if frames[i]["restart"] != frames[k]["restart"]:
                break
            holds = np.flatnonzero(exclusive[i - 2 : i + 1].all(axis=0))
            if len(holds) and holds[0] != e["actor"]:
                if holds[0] == e["receiver"] and math.dist(states[i, :2], states[k, :2]) >= 0.55:
                    passes.append(
                        dict(
                            kick=k, receive=i, passer=keys[e["actor"]], receiver=keys[e["receiver"]]
                        )
                    )
                break
    goals = [e for e in events if e["event"] == "goal" and e["tick"] >= start]
    for goal in goals:
        x, y, _ = goal["crossing_xyz"]
        assert abs(x) > 4.5215 and abs(y) < 0.5 - 0.0215
        assert (x * log["sign"] > 0) == (goal["team"] == "blue")
    # Count a new fast, close approach on entry or after disengaging.
    delta = xyz[:, 0:5, None, :2] - xyz[:, None, 5:, :2]
    gap = np.linalg.norm(delta, axis=-1)
    closing = (gap[:-1] - gap[1:]) / log["dt"]
    danger = (gap[1:] < 0.32) & (closing > 0.85)
    for i in range(1, len(frames)):
        if frames[i]["restart"] != frames[i - 1]["restart"] or i < start:
            danger[i - 1] = False
    entries = danger & ~np.concatenate((np.zeros_like(danger[:1]), danger[:-1]))
    return dict(
        attempts=sum(e["event"] == "pass" and e["tick"] >= start for e in kicks),
        passes=passes,
        goals=goals,
        fast_encounters=int(entries.sum()),
    )
