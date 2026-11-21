"""Choose distinct finalists that repair the known turnover, using training returns only."""

import argparse
from pathlib import Path
from run_recovery_attack import read, save


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("parent", type=Path)
    p.add_argument("regression", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("leagues", type=Path, nargs="+")
    p.add_argument(
        "--reception-regression",
        type=Path,
        help="Include receiver-only candidates that repaired the same recorded first touch",
    )
    a = p.parse_args()
    parent = read(a.parent)
    historical = read(a.regression / "launch.json")["models"][:-1]
    known = read(a.regression / "regression.json")
    finalists = [parent]
    for directory in a.leagues:
        cfg = read(directory / "launch.json")
        stats = read(directory / "returns.json")
        assert all(blue == yellow for _, blue, yellow in cfg["schedule"])
        eligible = [
            i
            for i, model in enumerate(cfg["models"])
            if model not in finalists and known[historical.index(model)]["qualified"]
        ]
        if eligible:
            j = max(
                eligible,
                key=lambda i: (stats[i]["goals"], stats[i]["shields"], -stats[i]["interceptions"]),
            )
            finalists.append(cfg["models"][j])
    if a.reception_regression:
        cfg = read(a.reception_regression / "launch.json")
        for model, row in zip(
            cfg["models"][:-1], read(a.reception_regression / "regression.json"), strict=True
        ):
            if row["qualified"] and model not in finalists:
                finalists.append(model)
    if len(finalists) < 2:
        raise ValueError("No new team reproduced a shielded goal against the original opponent")
    save(a.output, finalists)


if __name__ == "__main__":
    main()
