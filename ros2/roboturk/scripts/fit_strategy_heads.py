#!/usr/bin/env python3
"""Fit choice-conditioned strategic heads from retained native training datasets."""

import argparse
from pathlib import Path
from roboturk.strategy_adapt import fit_heads


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data", nargs="+", type=Path, required=True)
    p.add_argument("--parent", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--seed", type=int, required=True)
    p.add_argument("--epochs", type=int, default=250)
    p.add_argument("--geometry", action="store_true")
    a = p.parse_args()
    assert not a.output.exists() and not a.output.with_suffix(".json").exists()
    fit_heads(a.data, a.parent, a.output, a.seed, a.epochs, a.geometry)


if __name__ == "__main__":
    main()
