"""Fit a shallow motor selector from native 3v3 opening returns."""

import argparse
import json
from pathlib import Path
import numpy as np

from roboturk.touch_context import select_motor


def leaf(values, actions, parent, strength):
    n = len(parent)
    counts = np.bincount(actions, minlength=n)
    sums = np.bincount(actions, weights=values, minlength=n)
    means = (sums + strength * parent) / np.maximum(1e-6, counts + strength)
    j = int(np.argmax(means))
    return {"action": j}, means[j], means


def fit(x, choices, reward, n, depth, minimum, penalty):
    _, _, prior = leaf(reward, choices, np.zeros(n), 1.0)

    def split(x, choices, reward, remaining):
        node, score, _ = leaf(reward, choices, prior, 3.5)
        if remaining == 0:
            return node
        best = None
        for j in range(x.shape[1]):
            for value in np.unique(np.quantile(x[:, j], [0.20, 0.35, 0.50, 0.65, 0.80])):
                # Use thresholds at physical measurement resolution, including around zero.
                step = (
                    0.03,
                    0.03,
                    0.03,
                    0.003,
                    0.003,
                    0.003,
                    0.003,
                    0.003,
                    0.003,
                    0.03,
                    0.03,
                    0.2,
                    0.2,
                    0.03,
                    0.03,
                    0.03,
                    0.03,
                )[j]
                value = (np.floor(value / step) + 0.5) * step
                mask = x[:, j] <= value
                if (
                    min(np.bincount(choices[mask], minlength=n)) < minimum
                    or min(np.bincount(choices[~mask], minlength=n)) < minimum
                ):
                    continue
                parts = [leaf(reward[t], choices[t], prior, 3.5) for t in (mask, ~mask)]
                expected = mask.mean() * parts[0][1] + (1 - mask.mean()) * parts[1][1]
                if expected > score + penalty and (best is None or expected > best[0]):
                    best = expected, j, float(value), mask
        if best is None:
            return node
        _, j, value, mask = best
        return dict(
            feature=j,
            threshold=value,
            left=split(x[mask], choices[mask], reward[mask], remaining - 1),
            right=split(x[~mask], choices[~mask], reward[~mask], remaining - 1),
        )

    return split(x, choices, reward, depth)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output", type=Path)
    p.add_argument("leagues", type=Path, nargs="+")
    args = p.parse_args()
    rows = []
    policy = None
    for fold, d in enumerate(args.leagues):
        cfg = json.loads((d / "launch.json").read_text())
        if policy is None:
            policy = cfg["models"][0].copy()
        for i, (seed, a, b) in enumerate(cfg["schedule"]):
            game = json.loads((d / f"game-{i:03}.json").read_text())
            assert a == b and game["seed"] == seed
            assert cfg["models"][a]["carrier_bank"] == policy["carrier_bank"]
            arm = cfg["models"][a]["carrier_tree"]["action"]
            context = game["opening_context"]
            if context is None:
                continue
            assert set(game["motor_choices"].values()) == {arm}
            r = game["opening"]
            rows.append((fold, context, arm, r["qualified"], r["prepass_loss"]))
    n = len(policy["carrier_bank"])
    groups = np.array([r[0] for r in rows])
    x = np.array([r[1] for r in rows])
    actions = np.array([r[2] for r in rows])
    goals = np.array([r[3] for r in rows], float)
    losses = np.array([r[4] for r in rows], float)
    reward = goals - 3.0 * losses
    models = []
    fits = []
    # Use the same trained strategies in native validation. Logged off-policy
    # returns can be noisy when warm simulation resets produce different first touches.
    for depth, minimum, penalty in [(0, 10, 0.04), (1, 10, 0.035), (2, 8, 0.025), (2, 5, 0.012)]:
        tree = fit(x, actions, reward, n, depth, minimum, penalty)
        if tree in [m["carrier_tree"] for m in models]:
            continue
        validation = []
        for fold in np.unique(groups):
            train = groups != fold
            test = ~train
            hold_tree = fit(x[train], actions[train], reward[train], n, depth, minimum, penalty)
            used = test & (np.array([select_motor(hold_tree, v) for v in x]) == actions)
            validation.append(
                dict(
                    agreements=int(used.sum()),
                    goals=int(goals[used].sum()),
                    losses=int(losses[used].sum()),
                )
            )
        models.append(policy | dict(carrier_tree=tree))
        fits.append(
            dict(
                tree=tree,
                depth=depth,
                min_per_action=minimum,
                split_penalty=penalty,
                logged_validation=validation,
            )
        )
    args.output.mkdir(parents=True, exist_ok=False)
    for name, obj in [
        ("models", models),
        (
            "fits",
            dict(
                records=len(x), folds=len(args.leagues), reward="goal - 3 * prepass_loss", fits=fits
            ),
        ),
    ]:
        (args.output / f"{name}.json").write_text(json.dumps(obj, indent=2) + "\n")


if __name__ == "__main__":
    main()
