#!/usr/bin/env python3
"""Retain every generation and rollout, and stage the last self-play snapshot."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    if (args.output / "freeze.json").exists():
        raise ValueError("Refusing to modify a frozen release")
    league = json.loads((args.source / "league.json").read_text())
    if len(league["snapshots"]) < 3 or len(league["updates"]) < 8:
        raise ValueError("At least two complete four-role generations are required")
    for item in league["updates"]:
        if item["ppo_steps"] < 32768 or item["actor_before"] == item["actor_after"]:
            raise ValueError("A role did not receive a substantive self-play actor update")
    args.output.mkdir(parents=True, exist_ok=True)
    copied = []
    for file in sorted(args.source.rglob("*")):
        rel = file.relative_to(args.source)
        if file.suffix not in (".json", ".zip"):
            continue
        if (
            not rel.parts[0].startswith("round")
            and rel.parts[0]
            not in (
                "warm-start",
                "teacher",
                "dagger",
                "round1",
                "round2",
                "cross-play",
            )
            and file.name
            not in (
                "teacher-collection.json",
                "dagger-collection.json",
                "teacher-development.json",
                "dagger-development.json",
                "league.json",
                "bootstrap-bundle.json",
                "bootstrap-runtime.json",
                "training-progress.json",
                "cross-play-matrix.json",
            )
        ):
            continue
        dest = args.output / "artifacts" / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(file, dest)
        copied.append({"path": str(dest.relative_to(args.output)), "sha256": sha(dest)})
    selected = {}
    for name, bundle in (("final", league["snapshots"][-1]), ("bootstrap", league["snapshots"][0])):
        selected[name] = {}
        for role, entry in bundle.items():
            source = Path(entry["path"])
            if sha(source) != entry["sha256"]:
                raise ValueError("Historical file changed")
            dest = args.output / name / role / "policy.zip"
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, dest)
            shutil.copy2(source.with_suffix(".json"), dest.with_suffix(".json"))
            selected[name][role] = {
                "path": str(dest.relative_to(args.output)),
                "sha256": sha(dest),
                "manifest_sha256": sha(dest.with_suffix(".json")),
                "actor_sha256": entry["actor_sha256"],
            }
    result = {
        "rule": "last completed generation after development; no reserved-outcome selection",
        "generations": len(league["snapshots"]) - 1,
        "models": selected,
        "artifacts": copied,
    }
    (args.output / "selection.json").write_text(json.dumps(result, indent=2) + "\n")
    print(
        json.dumps(
            {
                "generations": result["generations"],
                "retained_files": len(copied),
                "models": selected,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
