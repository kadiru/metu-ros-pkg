#!/usr/bin/env python3
"""Retain every generation and rollout, and stage the last self-play snapshot."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_training(source, league):
    """Check the retained league and recorded training resets before release staging."""
    snapshots, rows = league["snapshots"], []
    bundles = snapshots + [b for item in league["updates"] for b in item["opponent_bundles"]]
    bundles += [item["preparation"] for item in league["updates"] if item.get("preparation")]
    for bundle in bundles:
        for entry in bundle.values():
            if sha(Path(entry["path"])) != entry["sha256"]:
                raise ValueError("Historical opponent bytes changed")
    for item in league["updates"]:
        g, role = item["generation"], item["role"]
        origin = item.get("preparation") or snapshots[g - 1]
        if (
            item["actor_before"] != origin[role]["actor_sha256"]
            or item["actor_after"] != snapshots[g][role]["actor_sha256"]
        ):
            raise ValueError("Actor update does not match its declared initialization and snapshot")
        if item.get("preparation"):
            receipt = json.loads(Path(origin[role]["path"]).with_name("training.json").read_text())
            if receipt["parent"] != snapshots[g - 1] or receipt["bundle"] != origin:
                raise ValueError("Prepared actor provenance differs from its parent generation")
        stage = source / f"round{g}"
        updates = json.loads((stage / (role + "-updates.json")).read_text())
        records = json.loads((stage / (role + "-rollouts.json")).read_text())
        if (
            len(updates) != item["updates"]
            or sum(x["rejected"] for x in updates) != item["rejected_updates"]
        ):
            raise ValueError("Update log disagrees with training metadata")
        episodes = records["episodes"] + records["failed_finish_prefixes"]
        for e in episodes:
            if not 2100000 <= e["seed"] < 2200000:
                raise ValueError("Training consumed a scenario outside the declared split")
            if not 0 <= e["opponent_pool_index"] < len(item["opponent_bundles"]):
                raise ValueError("Training opponent was outside the retained pool")
        rows.append(
            {
                "generation": g,
                "role": role,
                "ppo_steps": item["ppo_steps"],
                "complete_rollouts": len(records["episodes"]),
                "failed_finish_prefixes": len(records["failed_finish_prefixes"]),
                "accepted_updates": item["updates"] - item["rejected_updates"],
                "supervised_preparation": bool(item.get("preparation")),
                "sampled_pool_indices": sorted({e["opponent_pool_index"] for e in episodes}),
                "seed_range": [min(e["seed"] for e in episodes), max(e["seed"] for e in episodes)],
            }
        )
    if {(x["generation"], x["role"]) for x in rows} != {
        (g, r) for g in range(1, len(snapshots)) for r in snapshots[g]
    } or len(rows) != 4 * (len(snapshots) - 1):
        raise ValueError("Missing or duplicate generation/role update")
    return {
        "historical_hashes_verified": True,
        "reserved_training_resets": 0,
        "ppo_steps": sum(x["ppo_steps"] for x in rows),
        "roles": rows,
    }


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
    training_audit = validate_training(args.source, league)
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
                "spacing-repair",
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
                "cross-play-initial-matrix.json",
                "generation3-runtime.json",
                "spacing-repair-development.json",
                "cross-play-generation3-matrix.json",
            )
        ):
            continue
        dest = args.output / "artifacts" / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(file, dest)
        copied.append({"path": str(dest.relative_to(args.output)), "sha256": sha(dest)})
    audit_path = args.output / "training-audit.json"
    audit_path.write_text(json.dumps(training_audit, indent=2) + "\n")
    copied.append({"path": audit_path.name, "sha256": sha(audit_path)})
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
