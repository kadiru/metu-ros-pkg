#!/usr/bin/env python3
"""Fit a contest motor policy to native self-play returns across opponent groups."""

import argparse
import json
from pathlib import Path
import numpy as np


def read(path):
    return json.loads(path.read_text())


def fit(leagues, base):
    motor = np.asarray(base["motor"])
    # Fit a return model separately per action and opponent family. Keep
    # episodes equally weighted even when a contest lasts many decisions.
    groups = [[[] for _ in motor] for _ in range(2)]
    counts = []
    for league in leagues:
        updates = read(league / "updates.json")
        for i, update in enumerate(updates):
            game = read(league / f"game-{i:03}.json")
            row = game["contest_metrics"]["contests"]
            event = row[0] if row else {}
            win = event.get("winner", -1) // 3 == int(update["learner"] == "yellow")
            reward = 1 + 3 * ("received" in event) if win else -1
            assert reward == update["reward"]
            moves = [
                v
                for v in game["learned_actions"]
                if v["team"] == update["learner"]
                and v["head"] == "motor"
                and 4 * v["tick"] <= event.get("recovery", 100 * game["seconds"])
            ]
            group = int(update["opponent"] >= 0)
            for v in moves:
                k = v["action"]
                grad = np.array(v["gradient"])
                # Each row is a scalar times the same input vector. Use the
                # largest coefficient for a stable reconstruction at low entropy.
                j = np.argmax(np.abs(grad[:, 0]))
                x = grad[j] / grad[j, 0]
                groups[group][k].append((x, reward, 1 / len(moves)))
        counts.append(len(updates))
    critics = np.zeros((2, *motor.shape))
    support = np.zeros((2, len(motor)))
    for j, data in enumerate(groups):
        for k, examples in enumerate(data):
            if examples:
                x = np.array([e[0] for e in examples])
                y = np.array([e[1] + 1 for e in examples])
                w = np.array([e[2] for e in examples])
                ridge = np.diag([2, 12, 12, 12, 12])
                critics[j, k] = np.linalg.solve(x.T @ (x * w[:, None]) + ridge, x.T @ (w * y))
                support[j, k] = w.sum()
    # Optimize a conservative estimate of return across released and historical
    # opponents. Geometry slopes are shared, and the lower baseline for each
    # action guards against a move that only works in one opponent family.
    robust = critics.mean(0)
    robust[:, 0] = critics[:, :, 0].min(0)
    policy = dict(motor=robust.tolist(), outlet=base["outlet"], temperature=0.6)
    audit = dict(
        episodes=counts,
        effective_episodes_per_action=support.tolist(),
        critics=critics.tolist(),
        motor_method="group-conservative Monte Carlo policy improvement",
        outlet_source="first native league",
    )
    return policy, audit


def main():
    p = argparse.ArgumentParser()
    p.add_argument("output", type=Path)
    p.add_argument("leagues", type=Path, nargs="+")
    p.add_argument("--base", type=Path, default=Path("policies/contest_outlet_v1/policy.json"))
    a = p.parse_args()
    model, audit = fit(a.leagues, read(a.base))
    a.output.mkdir(exist_ok=True, parents=True)
    for name, content in (("policy.json", model), ("fit.json", audit)):
        (a.output / name).write_text(json.dumps(content, indent=2) + "\n")


if __name__ == "__main__":
    main()
