"""Freeze the self-play policy and package the native training branches before evaluation."""

import argparse
import gzip
import json
from pathlib import Path
from fit_takeaway import fit
from train_takeaway import CTRL, DEST, OLD, ROOT, WORLD, read, save, sha


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("training", type=Path)
    args = p.parse_args()
    d = args.training
    if (DEST / "freeze.json").exists():
        p.error("Existing published freeze")
    cfg = read(d / "launch.json")
    games = [read(d / f"game-{i:03}.json") for i in range(cfg["cases"])]
    base = read(OLD / "policy.json")
    previous = read(d / "policy.json")
    fit(d, games, cfg["starts"], cfg["references"], base)
    assert read(d / "policy.json") == previous
    cfg.pop("root")
    payload = dict(config=cfg, games=games)
    (DEST / "training.json.gz").write_bytes(
        gzip.compress(json.dumps(payload, separators=(",", ":")).encode(), mtime=0)
    )
    save(DEST / "policy.json", previous)
    save(DEST / "fit.json", read(d / "fit.json"))
    files = [
        WORLD,
        CTRL,
        OLD / "freeze.json",
        OLD / "policy.json",
        DEST / "policy.json",
        DEST / "fit.json",
        DEST / "protocol.json",
        ROOT / "src/roboturk_core/roboturk/takeaway.py",
        ROOT / "scripts/train_takeaway.py",
        ROOT / "scripts/fit_takeaway.py",
        Path(__file__).resolve(),
    ]
    save(DEST / "freeze.json", {str(f.relative_to(ROOT)): sha(f) for f in files})


if __name__ == "__main__":
    main()
