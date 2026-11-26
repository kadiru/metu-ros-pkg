#!/usr/bin/env python3
"""Select trained attacking actors on development, retaining every candidate."""

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
from zoneinfo import ZoneInfo


def stage(source, target, roles):
    if (target / "freeze.json").exists():
        raise ValueError("Cannot modify a frozen attacker release")
    selected = target / "selection.json"
    selection = json.loads(selected.read_text())["actors"] if selected.exists() else {}
    for role in roles:
        src, dst = source / role, target / role
        stages = dst / "stages"
        stages.mkdir(parents=True, exist_ok=True)
        candidates = {
            s: json.loads((src / (s + "-development.json")).read_text())
            for s in ("bc", "dagger", "policy")
        }
        best = max(
            candidates,
            key=lambda s: (
                candidates[s]["successes"],
                -candidates[s].get("possession_loss_episodes", 0),
                s == "policy",
                s == "dagger",
            ),
        )
        for p in src.iterdir():
            if p.suffix in (".zip", ".json"):
                shutil.copy2(p, stages / p.name)
        shutil.copy2(src / (best + ".zip"), dst / "policy.zip")
        meta = json.loads((src / (best + ".json")).read_text())
        meta.update(
            selected_stage=best,
            active_opponent_roles=["defender", "keeper"] if role == "transfer" else ["keeper"],
            selection_rule="highest development successes, fewest possession losses; PPO then DAgger breaks ties",
        )
        (dst / "policy.json").write_text(json.dumps(meta, indent=2) + "\n")
        (dst / "development.json").write_text(json.dumps(candidates[best], indent=2) + "\n")
        selection[role] = {
            "selected_stage": best,
            "development_successes": candidates[best]["successes"],
            "path": str(dst / "policy.zip"),
            "sha256": hashlib.sha256((dst / "policy.zip").read_bytes()).hexdigest(),
            "candidate_sha256": {
                s: hashlib.sha256((src / (s + ".zip")).read_bytes()).hexdigest() for s in candidates
            },
        }
    selected.write_text(
        json.dumps(
            {
                "created_at": datetime.now(ZoneInfo("America/Los_Angeles")).isoformat(),
                "actors": selection,
            },
            indent=2,
        )
        + "\n"
    )
    return selection


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--role", choices=("transfer", "finish"), action="append")
    args = parser.parse_args()
    print(
        json.dumps(stage(args.source, args.output, args.role or ["transfer", "finish"]), indent=2)
    )


if __name__ == "__main__":
    main()
