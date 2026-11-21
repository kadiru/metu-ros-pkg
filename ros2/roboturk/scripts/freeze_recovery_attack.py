#!/usr/bin/env python3
"""Replay the native self-play updates and bind the selected team to its source."""

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import numpy as np
from roboturk.attack_returns import AttackLearner
from run_recovery_attack import R, ROOT, WORLD, read, save


def replay(payload, warm):
    cfg = payload["launch"]
    fit = AttackLearner(warm)
    assert cfg["mode"] == "train" and len(payload["games"]) == cfg["cases"]
    for i, g in enumerate(payload["games"]):
        assert g["seed"] == cfg["first"] + i and g["learner"] == ("yellow" if i % 2 else "blue")
        assert isinstance(g["opponent"], int) and 0 <= g["opponent"] <= i // 12
        fit.update(g)
        if (i + 1) % 12 == 0:
            np.testing.assert_allclose(
                fit.policy(), payload["checkpoints"][str(i + 1)]["q"], atol=1e-10, rtol=1e-10
            )
    np.testing.assert_allclose(fit.policy(), payload["policy"]["q"], atol=1e-10, rtol=1e-10)
    np.testing.assert_array_equal(fit.counts, payload["policy"]["option_updates"])
    return dict(
        episodes=len(payload["games"]),
        option_updates=fit.counts.tolist(),
        checkpoints=len(payload["checkpoints"]),
    )


def archive(folder, name):
    payload = dict(
        launch=read(folder / "launch.json"),
        policy=read(folder / "policy.json"),
        games=[read(p) for p in sorted(folder.glob("game-*.json"))],
        checkpoints={
            str(int(p.stem.split("-")[1])): read(p) for p in folder.glob("checkpoint-*.json")
        },
    )
    warm = None
    if name != "initial":
        warm = json.loads(gzip.decompress((R / "initial.json.gz").read_bytes()))["policy"]["q"]
    receipt = replay(payload, warm)
    blob = gzip.compress(json.dumps(payload, separators=(",", ":")).encode(), mtime=0)
    (R / f"{name}.json.gz").write_bytes(blob)
    save(R / f"{name}-audit.json", receipt)
    print(receipt)


def freeze(candidate):
    payload = json.loads(gzip.decompress((R / "refined.json.gz").read_bytes()))
    model = read(candidate)
    assert any(
        np.array_equal(model["q"], p["q"])
        for p in [payload["policy"], *payload["checkpoints"].values()]
    )
    assert model["refined"] and "temperature" in model
    if (R / "freeze.json").exists():
        raise ValueError("Frozen policy already exists")
    save(R / "policy.json", model)
    files = [
        R / "policy.json",
        R / "protocol.json",
        R / "initial.json.gz",
        R / "refined.json.gz",
        WORLD,
        ROOT / "scripts/run_recovery_attack.py",
        ROOT / "scripts/audit_recovery_attack.py",
        Path(__file__).resolve(),
        ROOT
        / "src/roboturk_ros/webots/controllers/roboturk_recovery_attack/roboturk_recovery_attack.py",
    ]
    files.extend(
        ROOT / f"src/roboturk_core/roboturk/{name}.py"
        for name in ("recovery_attack", "recovery_conversion", "attack_returns")
    )
    files.extend(
        ROOT / "policies" / name
        for name in (
            "contest_outlet_v1/freeze.json",
            "contest_outlet_v1/isolation-freeze.json",
            "contest_outlet_v2/freeze.json",
        )
    )
    save(
        R / "freeze.json",
        {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("stage", choices=["initial", "refined", "freeze", "verify"])
    p.add_argument("source", nargs="?", type=Path)
    a = p.parse_args()
    if a.stage == "verify":
        warm = None
        for name in ("initial", "refined"):
            data = json.loads(gzip.decompress((R / f"{name}.json.gz").read_bytes()))
            print(replay(data, warm))
            warm = data["policy"]["q"]
    elif a.stage == "freeze":
        freeze(a.source)
    else:
        archive(a.source, a.stage)


if __name__ == "__main__":
    main()
