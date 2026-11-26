"""Replay the known turnover against the original blue team for each training finalist."""

import argparse
from pathlib import Path
import numpy as np
from train_pressure_receive import (
    ROOT,
    PRIOR,
    R,
    WEBOTS,
    read,
    save,
    sha,
    isolated,
    pressure_result,
)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("league", type=Path)
    p.add_argument("folder", type=Path)
    p.add_argument("--webots", type=Path, default=WEBOTS)
    a = p.parse_args()
    d = a.folder.resolve()
    d.mkdir(parents=True, exist_ok=False)
    candidates = read(a.league)
    plan = read(R / "protocol.json")
    n = len(candidates)
    schedule = [[13300014, n, j] for j in range(n)]
    save(
        d / "launch.json",
        dict(
            root=str(ROOT),
            protocol=plan,
            models=candidates + [read(PRIOR / "policy-1.json")],
            cases=n,
            schedule=schedule,
            seconds=18,
            record=False,
            study=False,
            press=False,
            webots_sha256=sha(a.webots),
        ),
    )
    rows = []
    for j in range(n):
        isolated(d, j, a.webots)
        states = np.load(d / f"trace-{j:03}.npz")["states"]
        game = read(d / f"game-{j:03}.json")
        row = pressure_result(states, game, plan)
        assert row == game["opening"]
        rows.append(row)
    save(d / "regression.json", rows)
    print([r["outcome"] for r in rows])


if __name__ == "__main__":
    main()
