"""Local action values from actual 3v3 recovery rollouts with balanced exploration."""

import numpy as np

ACTIONS = ("pass", "dribble", "shoot")
# Normalize field scales and give shot obstruction and receiver space comparable influence.
SCALE = np.array(
    [1.0, 0.85, 0.7, 1.25, 0.9, 0.9, 1.2, 1.15, 0.9, 0.75, 0.65, 0.5, 0.5, 0.85, 0.85, 0.8, 0.8]
)


def observation(base, actor, world):
    c, s = np.cos(actor.yaw), np.sin(actor.yaw)
    dx, dy = world.ball.x - actor.x, world.ball.y - actor.y
    return np.r_[
        base,
        np.clip((dx * c + dy * s - 0.115) / 0.035, -1, 1),
        np.clip((-dx * s + dy * c) / 0.045, -1, 1),
        np.clip((world.ball.vx * c + world.ball.vy * s) / 0.8, -1, 1),
        np.clip((-world.ball.vx * s + world.ball.vy * c) / 0.8, -1, 1),
    ]


def training_example(game):
    rows = game["conversions"]
    if not rows or "recovery" not in rows[0]:
        return None
    e = rows[0]
    o = next(
        (v for v in game["options"] if v["recovery"] == e["recovery"] and v["team"] == e["team"]),
        None,
    )
    if not o:
        return None
    success = "goal" in e
    a = ACTIONS.index(o["action"])
    score = -1.0
    if success:
        # Comparable scoring opportunities favor retained attacking combinations.
        delay = (e["goal"] - e["recovery"]) * 0.01
        score = 5.5 - 0.08 * delay + 2.6 * bool(e["passes"]) + 1.25 * (e["dribble_m"] >= 0.18)
    elif a == 0 and any(
        p["result"] == "complete" and p["team"] == o["team"] and o["tick"] <= p["tick"] < o["until"]
        for p in game["passes"]
    ):
        score = -0.6
    return dict(
        x=o["opening_x"],
        action=a,
        value=score,
        scored=success,
        assisted=bool(e.get("passes")),
        carried=e.get("dribble_m", 0) >= 0.18,
        seed=game["seed"],
    )


def values(model, x):
    scores = []
    for a in range(3):
        data = model["groups"][a]
        rows = np.asarray(data["x"])
        y = np.asarray(data["y"])
        scale = SCALE
        if rows.shape[1] > len(SCALE):
            extra = (
                np.array([2.4, 1.0, 1.65, 1.5, 0.65, 1.3])
                if a == 0
                else np.array([0.25, 0.2, 0.2, 0.25, 0.1, 0.25])
            )
            scale = np.r_[SCALE * (0.6 if a == 0 else 1), extra]
        dist = np.sum(((rows - x) * scale) ** 2, axis=1)
        n = min(
            len(y),
            model.get("pass_neighbors", model["neighbors"]) if a == 0 else model["neighbors"],
        )
        if a == 0:
            y = y + model.get("pass_success_bonus", 0.0) * (y > 0)
        nearest = np.argsort(dist)[:n]
        bw = max(0.12, float(dist[nearest[-1]]) * 0.45)
        weight = np.exp(-(dist[nearest] - dist[nearest[0]]) / bw)
        scores.append(float(weight @ y[nearest] / weight.sum()))
    return np.array(scores)


def fit(examples, neighbors=18):
    groups = []
    for a in range(3):
        data = [e for e in examples if e["action"] == a]
        groups.append(dict(x=[e["x"] for e in data], y=[e["value"] for e in data]))
    return dict(groups=groups, neighbors=neighbors)
