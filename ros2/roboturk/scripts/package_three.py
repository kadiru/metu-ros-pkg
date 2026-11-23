#!/usr/bin/env python3
"""Check native self-play updates, stage the final policies, and freeze their dependencies."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from roboturk.three_recipient import train_update

ROOT = Path(__file__).resolve().parents[1]
RELEASE = ROOT / "policies/recipient_3v3_v1"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stage(run, seed):
    training = json.loads((run / "training.json").read_text())
    launch = json.loads((run / "launch.json").read_text())
    assert launch["seed"] == training["seed"] == seed
    assert launch["seconds"] == 60 and launch["matches"] == len(training["updates"])
    weights = np.asarray(training["initial"])
    outcomes, counts = Counter(), Counter()
    for i, update in enumerate(training["updates"]):
        game = json.loads((run / f"game-{i:02}.json").read_text())
        assert game["seed"] == launch["first"] + i
        assert 7100000 <= game["seed"] < 7300000
        assert update["opponent_checkpoint"] <= i // 4
        team = update["learner"]
        assert team == ("yellow" if i % 2 else "blue")
        choices = [p for p in game["passes"] if p["team"] == team]
        assert len(choices) == update["choices"]
        np.testing.assert_allclose(weights, update["before"], atol=1e-10)
        for p in choices:
            f0, f1 = np.array(p["features"])
            expected = 1 / (1 + np.exp(-np.clip((f1 - f0) @ weights, -30, 30)))
            assert np.isclose(p["probability"], expected)
            assert p["reward"] == (1 if p["result"] == "complete" else -1)
        weights = train_update(weights, choices, 2.0)
        np.testing.assert_allclose(weights, update["after"], atol=1e-10)
        counts[team] += len(choices)
        outcomes.update(p["result"] for p in choices)
    np.testing.assert_allclose(weights, training["final"])
    assert all(counts[t] > 0 for t in ("blue", "yellow"))
    assert np.linalg.norm(weights - training["initial"]) > 0.01
    dest = RELEASE / "training" / str(seed)
    if dest.exists():
        for source in run.iterdir():
            if source.is_file() and digest(source) != digest(dest / source.name):
                raise ValueError(f"Existing training evidence differs: {source.name}")
    else:
        shutil.copytree(run, dest)
    return weights.tolist(), dict(
        matches=len(training["updates"]),
        decisions=dict(counts),
        outcomes=dict(outcomes),
        policy_l2_change=float(np.linalg.norm(weights - training["initial"])),
    )


def freeze():
    dest = RELEASE / "freeze.json"
    if dest.exists():
        raise ValueError("Existing freeze belongs to the recorded qualification")
    dependencies = [
        RELEASE / "selectors.json",
        RELEASE / "protocol.json",
        ROOT / "scripts/run_three.py",
        ROOT / "scripts/audit_three.py",
        ROOT / "src/roboturk_ros/webots/worlds/ssl_three.wbt",
        ROOT / "src/roboturk_ros/webots/controllers/roboturk_three/roboturk_three.py",
        ROOT / "src/roboturk_ros/webots/protos/SslRobot.proto",
        ROOT / "src/roboturk_ros/webots/protos/SslBall.proto",
        ROOT / "src/roboturk_ros/webots/protos/SslDivisionBField.proto",
    ]
    dependencies.extend((ROOT / "src/roboturk_core/roboturk").glob("*.py"))
    motor = ROOT / "policies/strategy_self_play_v1/preparation/pass-motor-v5/policy.pt"
    dependencies.extend([motor, motor.with_suffix(".json")])
    dest.write_text(
        json.dumps(
            dict(files={str(p.relative_to(ROOT)): digest(p) for p in sorted(dependencies)}),
            indent=2,
        )
        + "\n"
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--leagues", nargs=2, type=Path)
    p.add_argument("--freeze", action="store_true")
    a = p.parse_args()
    if a.leagues:
        models, audits = {}, {}
        for seed, path in zip((901, 902), a.leagues, strict=True):
            models[str(seed)], audits[str(seed)] = stage(path, seed)
        for filename, data in [("selectors.json", models), ("training-audit.json", audits)]:
            (RELEASE / filename).write_text(json.dumps(data, indent=2) + "\n")
    if a.freeze:
        freeze()


if __name__ == "__main__":
    main()
