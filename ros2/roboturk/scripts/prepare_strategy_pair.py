#!/usr/bin/env python3
"""Prepare two independently initialized selectors from one native training set."""

import argparse
from pathlib import Path
from roboturk.strategy_adapt import fit_heads
from roboturk.strategy_training import write, digest


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data", type=Path, required=True)
    p.add_argument("--parents", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--epochs", type=int, default=250)
    a = p.parse_args()
    assert not a.output.exists()
    a.output.mkdir(parents=True)
    for seed in (811, 822):
        fit_heads(
            [a.data],
            a.parents / f"seed{seed}/refit.zip",
            a.output / f"seed{seed}/initial.zip",
            seed,
            a.epochs,
        )
    write(
        a.output / "complete.json",
        {
            "training_seeds": [811, 822],
            "dataset": str(a.data),
            "dataset_sha256": digest(a.data),
            "epochs": a.epochs,
            "parents": str(a.parents),
        },
    )


if __name__ == "__main__":
    main()
