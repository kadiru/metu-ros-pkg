"""Estimate the return of shared-ball maneuvers before a clean opening recovery."""

import argparse
from pathlib import Path
import numpy as np

from train_recovery_touch import ROOT, read, save


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("league", type=Path)
    p.add_argument("carrier", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    anchor = read(ROOT / "policies/contest_outlet_v2/policy.json")["jaw"]
    q = np.asarray(anchor["q"])
    grams = np.tile(np.diag([2.0, 4.0, 4.0, 4.0, 4.0, 6.0, 6.0, 5.0]), (len(q), 1, 1))
    sums = np.zeros_like(q)
    support = np.zeros(len(q), dtype=int)
    for file in sorted(args.league.glob("game-*.json")):
        g = read(file)
        r = g["opening"]
        chain = r["chain"]
        end = chain.get("recovery", min(600, int(g["seconds"] * 100))) / 4
        for team in ("blue", "yellow"):
            moves = [d for d in g["jaw_choices"] if d["team"] == team and d["tick"] <= end]
            if not moves:
                continue
            owner = chain.get("team") == team
            value = (
                0.7 + 1.4 * r["qualified"] - 2.5 * r["prepass_loss"]
                if owner
                else -0.7 - 0.4 * r["qualified"] + 0.6 * r["prepass_loss"]
            )
            if "winner" not in chain:
                value = -1.0
            weights = np.exp(-0.045 * (end - np.array([d["tick"] for d in moves])))
            weights /= max(1e-8, weights.sum())
            for w, d in zip(weights, moves, strict=True):
                x = np.asarray(d["context"])
                i = d["action"]
                grams[i] += w * np.outer(x, x)
                sums[i] += w * value * x
                support[i] += 1
    residual = np.stack([np.linalg.solve(g, b) for g, b in zip(grams, sums, strict=True)])
    carrier = read(args.carrier)
    models = [carrier]
    for gain, temp in ((0.55, 0.45), (1.0, 0.42), (1.6, 0.48), (0.9, 0.34)):
        models.append(
            carrier | dict(recovery_jaw=dict(q=(q + gain * residual).tolist(), temperature=temp))
        )
    args.output.mkdir(parents=True, exist_ok=False)
    save(args.output / "models.json", models)
    save(
        args.output / "fit.json",
        dict(
            action_support=support.tolist(),
            estimated_return=residual.tolist(),
            games=len(list(args.league.glob("game-*.json"))),
        ),
    )


if __name__ == "__main__":
    main()
