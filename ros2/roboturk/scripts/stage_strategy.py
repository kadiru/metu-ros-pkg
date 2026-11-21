#!/usr/bin/env python3
"""Retain native attempts with lossless compression and exact source snapshots."""

import argparse
import gzip
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RELEASE = ROOT / "policies/strategy_self_play_v1"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n")


def package(source):
    source = source.resolve()
    receipt = RELEASE / "preparation" / (source.name + "-transport.json")
    if receipt.exists():
        raise ValueError("Retain previous packaged attempt")
    files = list(source.rglob("*")) if source.is_dir() else []
    for suffix in (
        ".json",
        ".error.json",
        ".launch.json",
        ".log",
        ".mp4",
        ".movie.json",
        ".audit.json",
        ".inference.json",
    ):
        p = source.with_suffix(suffix)
        if p.exists():
            files.append(p)
    files += list(source.with_suffix(".launch").rglob("*"))
    rows = []
    for p in sorted(set(files)):
        if not p.is_file() or "__pycache__" in p.parts:
            continue
        target = RELEASE / "preparation" / p.relative_to(source.parent)
        data = p.read_bytes()
        packed = data
        if p.suffix == ".json" and len(data) > 500000:
            packed = gzip.compress(data, mtime=0)
            assert gzip.decompress(packed) == data
            target = target.with_suffix(".json.gz")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(packed)
        rows.append(
            {
                "source": str(p.relative_to(ROOT)),
                "path": str(target.relative_to(ROOT)),
                "payload_sha256": hashlib.sha256(data).hexdigest(),
                "sha256": sha(target),
                "original_bytes": len(data),
                "bytes": len(packed),
            }
        )
    write(receipt, rows)
    print(source.name, len(rows), "retained files", sum(r["bytes"] for r in rows), "bytes")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("source", type=Path)
    package(p.parse_args().source)


if __name__ == "__main__":
    main()
