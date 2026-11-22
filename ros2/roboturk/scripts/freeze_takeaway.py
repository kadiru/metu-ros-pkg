"""Freeze the self-play policy and package the native training branches before evaluation."""

import argparse
import gzip
import json
from pathlib import Path
from fit_takeaway import fit, fit_rows, branch_returns
from roboturk.takeaway import MODES
from roboturk.takeaway_outlet import EXTRA
from train_takeaway import CTRL, DEST, OLD, ROOT, WORLD, read, save, sha


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("training", type=Path)
    p.add_argument("extended", type=Path)
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
    extension = read(args.extended / "launch.json")
    added = [read(args.extended / f"game-{i:03}.json") for i in range(extension["cases"])]
    additional = branch_returns(
        args.extended,
        added,
        extension["starts"],
        extension["references"],
        range(len(MODES), len(MODES) + len(EXTRA)),
    )
    primary = read(d / "fit.json")
    combined = []
    for row, extra in zip(primary["rows"], additional, strict=True):
        assert {k: v for k, v in row.items() if k != "returns"} == {
            k: v for k, v in extra.items() if k != "returns"
        }
        combined.append(row | dict(returns=row["returns"] + extra["returns"]))
    fit_rows(DEST, combined, MODES + EXTRA, base)
    save(DEST / "first-fit.json", primary)
    for name, config, data in [("training", cfg, games), ("extended", extension, added)]:
        config.pop("root")
        payload = dict(config=config, games=data)
        (DEST / f"{name}.json.gz").write_bytes(
            gzip.compress(json.dumps(payload, separators=(",", ":")).encode(), mtime=0)
        )
    files = [
        WORLD,
        CTRL,
        OLD / "freeze.json",
        OLD / "policy.json",
        DEST / "policy.json",
        DEST / "fit.json",
        DEST / "protocol.json",
        ROOT / "src/roboturk_core/roboturk/takeaway.py",
        ROOT / "src/roboturk_core/roboturk/takeaway_outlet.py",
        ROOT / "scripts/train_takeaway.py",
        ROOT / "scripts/fit_takeaway.py",
        Path(__file__).resolve(),
    ]
    save(DEST / "freeze.json", {str(f.relative_to(ROOT)): sha(f) for f in files})


if __name__ == "__main__":
    main()
