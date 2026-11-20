#!/usr/bin/env python3
"""Stage both trained leagues, then freeze every deployment dependency before testing."""

import argparse
from collections import Counter
from datetime import datetime
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import shutil
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
RELEASE = ROOT / "policies/strategy_self_play_v1"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n")


def stage(source, seed):
    from roboturk.strategy_training import load
    from roboturk.self_play_training import actor_hash
    from roboturk.strategy_selfplay import target_hash

    league = json.loads((source / "league.json").read_text())
    assert league["seed"] == seed and len(league["updates"]) == 2
    assert not (RELEASE / f"seed{seed}").exists(), "Retain selected model artifacts"
    previous = Path(league["history"][0]["path"])
    rows = []
    initial = load(previous)
    model_sources = [("initial", previous)]
    for gen, meta in enumerate(league["updates"], 1):
        folder = source / f"generation{gen}"
        path = folder / "policy.zip"
        model = load(path)
        resets = json.loads((folder / "resets.json").read_text())
        role_counts = Counter(r["learner"] for r in resets)
        pool_counts = Counter(r["opponent"] for r in resets)
        assert set(role_counts) == {"blue0", "blue1", "yellow0", "yellow1"}
        assert set(pool_counts) == set(range(gen))
        assert all(6200000 <= r["seed"] < 6300000 for r in resets)
        assert all(
            r["opponent_sha256"] == meta["opponent_pool"][r["opponent"]]["sha256"] for r in resets
        )
        assert meta["steps"] == 32768 and meta["accepted_updates"] > 0
        assert meta["actor_after"] == actor_hash(model) != meta["actor_before"]
        assert (
            meta["targets_before"]
            == meta["targets_after"]
            == target_hash(model)
            == target_hash(initial)
        )
        assert meta["actor_before"] == actor_hash(load(previous))
        assert meta["sampled_choices"] and sum(meta["sampled_choices"].values()) == 32768
        rows.append(
            {
                "generation": gen,
                "steps": meta["steps"],
                "accepted_updates": meta["accepted_updates"],
                "attempted_updates": meta["attempted_updates"],
                "roles": dict(role_counts),
                "opponent_samples": dict(pool_counts),
                "actor_before": meta["actor_before"],
                "actor_after": meta["actor_after"],
                "target_function_sha256": target_hash(model),
                "deterministic_probe_choice_changes": meta["deterministic_probe_choice_changes"],
                "exploratory_choice_changes": meta["exploratory_choice_changes"],
                "checkpoint_sha256": sha(path),
                "passed": True,
            }
        )
        model_sources.append((f"generation{gen}", path))
        previous = path
    for label, path in model_sources:
        target = RELEASE / f"seed{seed}" / label / "policy.zip"
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        shutil.copy2(path.with_suffix(".json"), target.with_suffix(".json"))
        assert sha(target) == sha(path)
    write(
        RELEASE / f"seed{seed}/training-audit.json",
        {
            "seed": seed,
            "source": str(source.relative_to(ROOT)),
            "passed": True,
            "generations": rows,
        },
    )
    return rows


def freeze():
    destination = RELEASE / "freeze.json"
    assert not destination.exists(), "Never overwrite a published freeze"
    for seed in (811, 822):
        assert json.loads((RELEASE / f"seed{seed}/training-audit.json").read_text())["passed"]
    files = list((ROOT / "src/roboturk_core/roboturk").glob("*.py"))
    files += list((ROOT / "src/roboturk_ros/webots/controllers/roboturk_strategy").glob("*.py"))
    files += list((ROOT / "src/roboturk_ros/webots/protos").glob("*.proto"))
    files += [
        ROOT / "src/roboturk_ros/webots/worlds/ssl_strategy.wbt",
        ROOT / "requirements-macos-validation.txt",
        ROOT / "pyproject.toml",
        RELEASE / "protocol.json",
    ]
    files += [
        ROOT / "scripts" / p
        for p in (
            "run_strategy.py",
            "audit_strategy.py",
            "qualify_strategy.py",
            "freeze_strategy.py",
            "stage_strategy.py",
            "build_strategy_gif.py",
            "replay_strategy.py",
        )
    ]
    for seed in (811, 822):
        files += list((RELEASE / f"seed{seed}").rglob("*"))
    dependencies = [ROOT / "policies/continuous_play_v2/seed611/generation2/policy.zip"]
    dependencies += [
        ROOT / f"policies/tdp_skills_v1/{kind}/policy.zip"
        for kind in ("receive", "redirect", "clear")
    ]
    for p in dependencies:
        files += [p, p.with_suffix(".json")]
    files = sorted({p for p in files if p.is_file()})
    packages = {d.metadata["Name"]: d.version for d in importlib.metadata.distributions()}
    write(
        destination,
        {
            "created_at": datetime.now(ZoneInfo("America/Los_Angeles")).isoformat(),
            "selection": "Final completed generation per seed, selected before reserved evaluation",
            "python": platform.python_version(),
            "platform": platform.platform(),
            "packages": packages,
            "webots": "R2025a nightly 2026-11-17 e5d1122",
            "webots_executable_sha256": sha(
                ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots"
            ),
            "files": {str(p.relative_to(ROOT)): sha(p) for p in files},
        },
    )
    print("Frozen", len(files), "files", sha(destination))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--league811", type=Path)
    p.add_argument("--league822", type=Path)
    p.add_argument("--freeze", action="store_true")
    a = p.parse_args()
    if a.freeze:
        assert a.league811 is None and a.league822 is None
        freeze()
    else:
        assert a.league811 is not None and a.league822 is not None
        for seed, source in ((811, a.league811), (822, a.league822)):
            print(seed, stage(source.resolve(), seed))


if __name__ == "__main__":
    main()
