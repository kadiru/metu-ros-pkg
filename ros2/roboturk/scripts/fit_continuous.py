#!/usr/bin/env python3
"""CPU-only supervised initialization/refinement from explicit training datasets."""

import argparse
from pathlib import Path
import json
import numpy as np
import torch
from roboturk.continuous_training import digest, fit
from roboturk.self_play_training import write


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dataset", type=Path, action="append", required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--source", type=Path)
    p.add_argument("--seed", type=int, required=True)
    p.add_argument("--epochs", type=int, default=320)
    args = p.parse_args()
    if args.output.exists():
        p.error("Preserve prior stages: output must be new")
    torch.set_num_threads(1)
    arrays, provenance = [], []
    for directory in args.dataset:
        receipt = json.loads((directory / "collection.json").read_text())
        assert receipt["backend"] == "webots_ideal_body_velocity"
        assert all(4200000 <= m["seed"] < 4220000 for m in receipt["matches"])
        path = directory / "data.npz"
        assert digest(path) == receipt["data_sha256"]
        d = np.load(path)
        arrays.append((d["observations"], d["actions"]))
        provenance.append(
            {
                "path": str(directory),
                "data_sha256": digest(path),
                "collection_sha256": digest(directory / "collection.json"),
                "observations": len(d["observations"]),
                "seed": receipt["seed"],
                "count": receipt["count"],
                "control": receipt["control"],
            }
        )
    args.output.mkdir(parents=True)
    np.savez_compressed(
        args.output / "data.npz",
        observations=np.concatenate([a[0] for a in arrays]),
        actions=np.concatenate([a[1] for a in arrays]),
    )
    write(
        args.output / "datasets.json",
        {
            "inputs": provenance,
            "source": str(args.source) if args.source else None,
            "source_sha256": digest(args.source) if args.source else None,
            "data_sha256": digest(args.output / "data.npz"),
        },
    )
    fit(args.output, args.seed, args.epochs, str(args.source) if args.source else None)


if __name__ == "__main__":
    main()
