"""Compare local opening estimates on self-play layouts excluded from the fit."""

import argparse
from collections import Counter
import numpy as np
from roboturk.attack_opening import fit, training_example, values
from run_recovery_attack import read, save
from pathlib import Path


def compare(games, neighbors, pass_neighbors=8, pass_success_bonus=0):
    data = [training_example(g) for g in games]
    data = [e for e in data if e]
    retained = []
    predictions = Counter()
    for fold in range(5):
        model = fit([e for e in data if e["seed"] % 5 != fold], neighbors)
        model.update(pass_neighbors=pass_neighbors, pass_success_bonus=pass_success_bonus)
        for e in data:
            if e["seed"] % 5 != fold:
                continue
            act = int(np.argmax(values(model, np.asarray(e["x"]))))
            predictions[act] += 1
            if act == e["action"]:
                retained.append(e)
    return dict(
        neighbors=neighbors,
        pass_neighbors=pass_neighbors,
        pass_success_bonus=pass_success_bonus,
        assisted=sum(e["assisted"] for e in retained),
        carried=sum(e["carried"] for e in retained),
        predictions=[predictions[i] for i in range(3)],
        matched=len(retained),
        converted=sum(e["scored"] for e in retained),
        by_action=[sum(e["scored"] and e["action"] == i for e in retained) for i in range(3)],
        mean_return=float(np.mean([e["value"] for e in retained])),
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("league", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    games = [read(f) for f in sorted(a.league.glob("game-*.json"))]
    rows = [
        compare(games, n, pn, b) for n in (8, 18, 36) for pn in (4, 10, 20) for b in (0, 3, 6, 9)
    ]
    diverse = [r for r in rows if min(r["assisted"], r["carried"]) >= 0.08 * r["matched"]]
    selected = max(diverse or rows, key=lambda r: r["mean_return"])
    model = read(a.league / "policy.json")
    model["opening"].update(
        {k: selected[k] for k in ("neighbors", "pass_neighbors", "pass_success_bonus")}
    )
    a.output.mkdir(exist_ok=False, parents=True)
    save(a.output / "policy.json", model)
    save(a.output / "selection.json", dict(folds=5, rows=rows, selected=selected))
    print(selected)


if __name__ == "__main__":
    main()
