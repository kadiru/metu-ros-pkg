#!/usr/bin/env python3
"""Losslessly package native attempts; freeze selected options before reserved use."""

import argparse
from datetime import datetime
import gzip
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import shutil
from zoneinfo import ZoneInfo

from audit_tdp import audit_file

ROOT = Path(__file__).resolve().parents[1]
RELEASE = ROOT / "policies/tdp_skills_v1"
KINDS = ("receive", "redirect", "clear")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n")


def package(source):
    source = source.resolve()
    destination = RELEASE / "preparation" / source.name
    if destination.exists():
        raise ValueError("Retain existing packaged attempts")
    rows = []
    files = list(source.rglob("*"))
    receipt = source.with_suffix(".launch.json")
    files += [receipt]
    files += [p for suffix in (".movie.json", ".mp4") if (p := source.with_suffix(suffix)).exists()]
    files += list(receipt.with_suffix("").rglob("*"))
    for path in sorted(files):
        if not path.is_file():
            continue
        target = RELEASE / "preparation" / path.relative_to(source.parent)
        raw = path.read_bytes()
        packed = raw
        if path.suffix == ".json" and len(raw) > 500000:
            packed = gzip.compress(raw, mtime=0)
            assert gzip.decompress(packed) == raw
            target = target.with_suffix(".json.gz")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(packed)
        rows.append(
            {
                "source": str(path.relative_to(ROOT)),
                "path": str(target.relative_to(ROOT)),
                "payload_sha256": hashlib.sha256(raw).hexdigest(),
                "sha256": sha(target),
                "original_bytes": len(raw),
                "bytes": len(packed),
            }
        )
    write(RELEASE / "preparation" / (source.name + "-transport.json"), rows)
    return destination


def training(source):
    from roboturk.dribble_refinement import AnchoredPPO
    from roboturk.tdp_training import actor_hash, descriptor

    evidence = {}
    for i, kind in enumerate(KINDS):
        folder = source / kind
        summary = json.loads((folder / "training-summary.json").read_text())
        assert summary["ppo_steps"] == 32768 and summary["accepted_updates"] > 0
        assert summary["actor_changed"] and summary["final_successes"] >= 57
        assert summary["dataset_sha256"] == sha(folder / "demonstrations.npz")
        model = AnchoredPPO.load(folder / "policy.zip", device="cpu")
        metadata = json.loads((folder / "policy.json").read_text())
        assert actor_hash(model) == metadata["actor_sha256"] != metadata["actor_before_ppo"]
        assert model.num_timesteps == 32768
        development = {}
        for stage in ("bc", "dagger", "policy"):
            development[stage] = audit_file(
                folder / (stage + "-development.json"), 5010000 + i * 200000, 60
            )
        target = RELEASE / kind
        target.mkdir(parents=True, exist_ok=True)
        for name in ("policy.zip", "policy.json"):
            if (target / name).exists():
                raise ValueError("Selected checkpoint already staged")
            shutil.copy2(folder / name, target / name)
        evidence[kind] = {
            "training": summary,
            "development": development,
            "selected": descriptor(target / "policy.zip", model),
        }
    package(source)
    write(RELEASE / "training-audit.json", evidence)


def freeze():
    destination = RELEASE / "freeze.json"
    if destination.exists():
        raise ValueError("Never overwrite a freeze")
    integration = RELEASE / "preparation/orchestration-v1"
    for i, kind in enumerate(KINDS):
        path = integration / (kind + ".json.gz")
        if not path.exists():
            path = path.with_suffix("")
        report = audit_file(path, 5040000 + i * 200000, 30)
        assert report["success_rate"] >= 0.95
        assert not any(
            report[k] for k in ("collisions", "field_exits", "own_goals", "premature_clearances")
        )
    files = list((ROOT / "src/roboturk_core/roboturk").glob("*.py"))
    files += list((ROOT / "src/roboturk_ros/webots/controllers/roboturk_tdp").glob("*.py"))
    files += list((ROOT / "src/roboturk_ros/webots/protos").glob("*.proto"))
    files += [ROOT / "src/roboturk_ros/webots/worlds/ssl_tdp.wbt"]
    files += list((ROOT / "scripts").glob("*tdp.py"))
    files += list((ROOT / "tests").glob("test_tdp*.py"))
    files += [ROOT / "tests/fixtures/tdp-native-baselines.json"]
    files += [p for p in RELEASE.rglob("*") if p.is_file() and p.suffix != ".md"]
    selected = {
        k: {
            "path": str((RELEASE / k / "policy.zip").relative_to(ROOT)),
            "sha256": sha(RELEASE / k / "policy.zip"),
            "metadata_sha256": sha(RELEASE / k / "policy.json"),
            "actor_sha256": json.loads((RELEASE / k / "policy.json").read_text())["actor_sha256"],
        }
        for k in KINDS
    }
    write(
        destination,
        {
            "contract": "roboturk.tdp_options.v1",
            "created_at": datetime.now(ZoneInfo("America/Los_Angeles")).isoformat(),
            "selection": "Final PPO checkpoints chosen before reserved cases; scripted coordinator",
            "models": selected,
            "dependencies": {
                n: version(n) for n in ("torch", "numpy", "gymnasium", "stable_baselines3")
            },
            "webots": {
                "build": "R2025a nightly 2026-11-17",
                "commit": "e5d1122a3cd59f007ee4b37c8f651b50e7ee83d2",
                "executable_sha256": sha(
                    ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots"
                ),
            },
            "files": {str(p.relative_to(ROOT)): sha(p) for p in sorted(set(files))},
        },
    )
    print("Frozen", len(set(files)), "files; SHA256", sha(destination))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=("training", "package", "freeze"))
    p.add_argument("source", nargs="?", type=Path)
    a = p.parse_args()
    if a.mode == "freeze":
        freeze()
    else:
        if a.source is None:
            p.error("Source attempt directory required")
        (training if a.mode == "training" else package)(a.source)


if __name__ == "__main__":
    main()
