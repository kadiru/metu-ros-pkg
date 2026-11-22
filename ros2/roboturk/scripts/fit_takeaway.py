"""Fit a small contextual maneuver policy to full native counterfactual returns."""

import numpy as np
from train_recovery_touch import save
from roboturk.takeaway import MODES, select


def learn(rows, depth):
    returns = np.array([r["returns"] for r in rows])
    # Keep the incumbent on ties so that interventions earn their place.
    action = int(returns.sum(axis=0).argmax())
    best = float(returns[:, action].sum())
    tree = dict(action=action)
    if depth == 0 or len(rows) < 8:
        return tree, best
    for j in range(len(rows[0]["x"])):
        values = sorted(set(r["x"][j] for r in rows))
        for a, b in zip(values, values[1:]):
            t = (a + b) / 2
            left = [r for r in rows if r["x"][j] <= t]
            right = [r for r in rows if r["x"][j] > t]
            if min(len(left), len(right)) < 4:
                continue
            left_tree, left_score = learn(left, depth - 1)
            right_tree, right_score = learn(right, depth - 1)
            score = left_score + right_score - 0.45
            if score > best + 1e-8:
                tree = dict(feature=j, threshold=t, left=left_tree, right=right_tree)
                best = score
    return tree, best


def fit(d, games, starts, refs, base):
    rows = []
    n = len(MODES)
    for a in range(0, len(games), n):
        target = starts[a]
        turns = []
        states = np.load(d / f"trace-{a:03}.npz")["states"]
        through = (target // 4 + 1) * 4
        for j, g in enumerate(games[a : a + n]):
            branch = np.load(d / f"trace-{a + j:03}.npz")["states"]
            assert np.array_equal(states[:through], branch[:through])
            row = next(
                r for r in g["recoveries"] if r["sample"] == target and r["kind"] == "takeaway"
            )
            (decision,) = g["takeaway_actions"]
            assert decision["sample"] == target and decision["action"] == j
            turns.append(float("goal" in row))
        rows.append(
            dict(
                seed=g["seed"],
                reference=refs[a],
                sample=target,
                x=decision["x"],
                returns=turns,
                matched_samples=through,
            )
        )
    seeds = sorted({r["seed"] for r in rows})
    validation = []
    for depth in (0, 1, 2):
        correct = 0
        for seed in seeds:
            tree, _ = learn([r for r in rows if r["seed"] != seed], depth)
            correct += sum(r["returns"][select(tree, r["x"])] for r in rows if r["seed"] == seed)
        validation.append(correct)
    depth = int(np.argmax(validation))
    tree, _ = learn(rows, depth)
    save(
        d / "fit.json",
        dict(
            options=MODES,
            rows=rows,
            held_out_goals=validation,
            depth=depth,
            fitted_goals=sum(r["returns"][select(tree, r["x"])] for r in rows),
            tree=tree,
        ),
    )
    save(d / "policy.json", base | dict(takeaway_tree=tree))
