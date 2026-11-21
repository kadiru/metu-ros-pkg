#!/usr/bin/env python3
"""Prepare two independently initialized selectors from one native training set."""

import argparse
from pathlib import Path
from roboturk.strategy_adapt import fit_heads
from roboturk.strategy_training import write, digest


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data", nargs="+", type=Path, required=True)
    p.add_argument("--parents", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--epochs", type=int, default=250)
    p.add_argument("--parent-name", default="refit.zip")
    p.add_argument("--geometry", action="store_true")
    a = p.parse_args()
    assert not a.output.exists()
    a.output.mkdir(parents=True)
    for seed in (811, 822):
        fit_heads(
            a.data,
            a.parents / f"seed{seed}" / a.parent_name,
            a.output / f"seed{seed}/initial.zip",
            seed,
            a.epochs,
            a.geometry,
        )
    write(
        a.output / "complete.json",
        {
            "training_seeds": [811, 822],
            "datasets": [{"path": str(p), "sha256": digest(p)} for p in a.data],
            "epochs": a.epochs,
            "parents": str(a.parents),
            "parent_name": a.parent_name,
            "geometry_features": a.geometry,
            "source_sha256": {
                str(p): digest(p)
                for p in [
                    Path(__file__),
                    *Path(__file__)
                    .resolve()
                    .parents[1]
                    .glob("src/roboturk_core/roboturk/strategy_*.py"),
                ]
            },
        },
    )


if __name__ == "__main__":
    main()
