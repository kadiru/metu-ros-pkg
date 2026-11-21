"""Monte Carlo option values fit to completed native possessions."""

import numpy as np

ACTIONS = ("pass", "dribble", "shoot")
PRIOR = np.array(
    [
        [-0.6, 0, 0, 0, -0.1, 0, 1.0, 1.2, 0.6, 0.4, 0],
        [0.42, -0.20, 0, -0.15, 0.32, 0, 0, 0, 0, 0, -0.18],
        [-0.6, 0.65, -0.2, 0.65, 0, 0.35, 0, 0, 0, 0, 1.35],
    ]
)


class AttackLearner:
    def __init__(self):
        self.gram = np.repeat((np.eye(PRIOR.shape[1]) * 4)[None], 3, axis=0)
        self.y = 4 * PRIOR.copy()
        self.counts = np.zeros(3, dtype=int)

    def policy(self):
        return np.stack([np.linalg.solve(a, b) for a, b in zip(self.gram, self.y, strict=True)])

    def update(self, game):
        goals = {(v["team"], v["recovery"]): v for v in game["conversions"] if "goal" in v}
        for o in game["options"]:
            if o["team"] != game["learner"]:
                continue
            a = ACTIONS.index(o["action"])
            goal = goals.get((o["team"], o["recovery"]))
            ticks = o["tick"]
            value = -1.0
            if goal and 4 * ticks <= goal["goal"]:
                value = 6 * np.exp(-0.13 * (goal["goal"] * 0.01 - ticks * 0.04))
            if a == 0:
                passed = next(
                    (
                        p
                        for p in game["passes"]
                        if p["team"] == o["team"]
                        and p["passer"] == o["actor"]
                        and ticks <= p["tick"] < o["until"]
                    ),
                    None,
                )
                if passed:
                    value += 0.36 if passed["result"] == "complete" else -0.55
            if a == 2 and not goal:
                value -= 0.35
            x = np.array(o["x"])
            self.gram[a] += np.outer(x, x)
            self.y[a] += x * value
            self.counts[a] += 1
