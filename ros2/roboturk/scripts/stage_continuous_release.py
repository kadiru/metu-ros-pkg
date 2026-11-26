#!/usr/bin/env python3
"""Stage independent historical leagues; freeze only after development has passed."""

import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
from zoneinfo import ZoneInfo
from roboturk.continuous_training import digest, descriptor, load
from roboturk.self_play_training import write
from roboturk.continuous_selfplay import motion_hash, decision_head_hash

ROOT = Path(__file__).resolve().parents[1]


def stage(source, release, seed):
    if (release / "freeze.json").exists():
        raise ValueError("Frozen release cannot be restaged")
    ledger = json.loads((source / f"seed{seed}/league.json").read_text())
    history, updates = ledger["snapshots"], ledger["updates"]
    assert ledger["training_seed"] == seed
    assert len(history) == 3 and len(updates) == 2
    staged, total, accepted = [], 0, 0
    for i, entry in enumerate(history):
        assert digest(entry["path"]) == entry["sha256"]
        model = load(entry["path"])
        assert descriptor(entry["path"], model)["actor_sha256"] == entry["actor_sha256"]
        target = release / f"seed{seed}" / f"generation{i}" / "policy.zip"
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(entry["path"], target)
        shutil.copy2(Path(entry["path"]).with_suffix(".json"), target.with_suffix(".json"))
        staged.append({**descriptor(target, model), "path": str(target.relative_to(release))})
    for i, update in enumerate(updates, 1):
        assert update["seed"] == seed and update["generation"] == i
        assert update["opponent_pool"] == history[:i] and update["teammate"] == history[i - 1]
        assert update["actor_before"] == history[i - 1]["actor_sha256"]
        assert update["actor_after"] == history[i]["actor_sha256"] != update["actor_before"]
        assert update["decision_head_before"] != update["decision_head_after"]
        assert sum(update["sampled_modes"].values()) == update["steps"]
        assert update["choices_different_from_teammate_policy"] > 0
        assert update["steps"] == 32768 and update["backend"] == "webots_ideal_body_velocity"
        assert update["backend_contract"] == "roboturk.webots.closest_kicker.v1"
        if update.get("categorical_decisions"):
            assert update["decisions_only"] and update["behavior_logit_scale"] == 3
        if update["decisions_only"]:
            previous_model, updated_model = load(history[i - 1]["path"]), load(history[i]["path"])
            assert motion_hash(previous_model) == motion_hash(updated_model)
            assert update["motion_before"] == update["motion_after"] == motion_hash(updated_model)
            assert decision_head_hash(updated_model) == update["decision_head_after"]
            assert 0 <= update["deterministic_mode_probe_changes"] <= update["mode_probe_rows"]
        base = source / f"seed{seed}" / f"round{i}"
        resets = json.loads((base / "resets.json").read_text())
        rolls = json.loads((base / "rollouts.json").read_text())
        opt = json.loads((base / "updates.json").read_text())
        assert len(resets) == update["reset_count"]
        assert len(rolls) <= len(resets) <= len(rolls) + 1
        assert all(4300000 <= r["seed"] < 4400000 for r in resets)
        assert {r["learner"] for r in resets} == {"blue0", "blue1", "yellow0", "yellow1"}
        assert all(r["opponent_sha256"] == history[r["opponent_index"]]["sha256"] for r in resets)
        assert [r["seed"] for r in rolls] == [r["seed"] for r in resets[: len(rolls)]]
        assert [r["learner"] for r in rolls] == [r["learner"] for r in resets[: len(rolls)]]
        assert len(opt) == update["attempted_updates"] == 32
        assert sum(not r["rejected"] for r in opt) == update["accepted_updates"] > 0
        total += update["steps"]
        accepted += update["accepted_updates"]
    for p in (source / f"seed{seed}").rglob("*.json"):
        target = release / "training" / f"seed{seed}" / p.relative_to(source / f"seed{seed}")
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, target)
    result = {
        "seed": seed,
        "generations": 2,
        "ppo_steps": total,
        "accepted_updates": accepted,
        "snapshots": staged,
        "source": str(source),
        "source_ledger_sha256": digest(source / f"seed{seed}/league.json"),
    }
    write(release / f"seed{seed}/selection.json", result)
    return result


def freeze(release, protocol):
    if (release / "freeze.json").exists():
        raise ValueError("Refusing to overwrite freeze")
    gates = json.loads(protocol.read_text())
    assert gates["primary_seed"] == 611 and gates["training_seeds"] == [611, 622]
    selections = [json.loads((release / f"seed{s}/selection.json").read_text()) for s in (611, 622)]
    if gates.get("runtime_action_projection"):
        for selection in selections:
            changed_choices = 0
            for generation in (1, 2):
                update = json.loads(
                    (
                        release
                        / "training"
                        / f"seed{selection['seed']}"
                        / f"round{generation}"
                        / "training.json"
                    ).read_text()
                )
                assert update["safety_contract"] == gates["safety_contract"]
                assert update["categorical_decisions"] and update["decisions_only"]
                changed_choices += update["deterministic_mode_probe_changes"]
            assert changed_choices > 0, (
                "Selected league must change actual deterministic behavior choices"
            )
    assert len({r["snapshots"][0]["actor_sha256"] for r in selections}) == 2
    assert sum(r["ppo_steps"] for r in selections) == 131072
    files = [p for p in ROOT.glob("src/**/*") if p.suffix in (".py", ".proto", ".wbt", ".ini")]
    files += list((ROOT / "scripts").glob("*continuous*.py"))
    files += [
        ROOT / "requirements-macos-validation.txt",
        ROOT / "docs/runtime-manifest.json",
    ]
    artifact_files = [
        p
        for p in release.rglob("*")
        if p.is_file() and p.suffix in (".zip", ".json", ".py", ".wbt", ".proto", ".ini")
    ]
    result = {
        "created_at": datetime.now(ZoneInfo("America/Los_Angeles")).isoformat(),
        "protocol": gates,
        "protocol_sha256": digest(protocol),
        "selection": selections,
        "runtime_sha256": {str(p.relative_to(ROOT)): digest(p) for p in sorted(files)},
        "artifact_sha256": {str(p.relative_to(release)): digest(p) for p in sorted(artifact_files)},
    }
    write(release / "freeze.json", result)
    return {
        "runtime_files": len(files),
        "artifacts": len(artifact_files),
        "created_at": result["created_at"],
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=("stage", "freeze"))
    p.add_argument("--release", type=Path, required=True)
    p.add_argument("--source", type=Path)
    p.add_argument("--seed", type=int, choices=(611, 622))
    p.add_argument("--protocol", type=Path)
    args = p.parse_args()
    if args.mode == "stage":
        assert args.source and args.seed
        result = stage(args.source.resolve(), args.release.resolve(), args.seed)
    else:
        assert args.protocol
        result = freeze(args.release.resolve(), args.protocol.resolve())
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
