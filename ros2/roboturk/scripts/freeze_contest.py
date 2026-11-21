#!/usr/bin/env python3
"""Verify the self-play updates and bind the selected controller to its inputs."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
from roboturk.contest_outlet import initial, apply_gradient

ROOT = Path(__file__).resolve().parents[1]
R = ROOT / "policies/contest_outlet_v1"


def read(path):
    return json.loads(path.read_text())


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("league", type=Path)
    args = parser.parse_args()
    league = args.league
    cfg = read(league / "launch.json")
    assert cfg["mode"] == "train"
    weights = initial(np.random.default_rng(cfg["seed"]))
    before = {k: v.copy() for k, v in weights.items()}
    counts = dict(motor=0, outlet=0, own_wins=0, own_chains=0, designated_chains=0)
    logs = read(league / "updates.json")
    assert len(logs) == cfg["matches"]
    for i, row in enumerate(logs):
        d = read(league / f"game-{i:03}.json")
        assert row["seed"] == d["seed"] == cfg["first"] + i
        team = "yellow" if i % 2 else "blue"
        assert team == row["learner"] and -1 <= row["opponent"] <= i // 8
        events = d["contest_metrics"]["contests"]
        e = events[0] if events else {}
        own = e.get("winner", -1) // 3 == (0 if team == "blue" else 1)
        ds = [
            v
            for v in d["learned_actions"]
            if v["team"] == team and v["tick"] <= e.get("recovery", cfg["seconds"] * 100) // 4
        ]
        assert len(ds) == row["decisions"]
        won_pass = own and "received" in e
        outlet = won_pass and e["receiver"] % 3 in [v["mate"] for v in ds if v["head"] == "outlet"]
        counts["own_wins"] += own
        counts["own_chains"] += won_pass
        counts["designated_chains"] += outlet
        assert row["reward"] == (1 + 3 * won_pass if own else -1)
        assert row["outlet_reward"] == ((3 if outlet else -1) if own else 0)
        for head in ("motor", "outlet"):
            actions = [v for v in ds if v["head"] == head]
            counts[head] += len(actions)
            apply_gradient(
                weights, actions, row["reward" if head == "motor" else "outlet_reward"], 0.65
            )
            np.testing.assert_allclose(weights[head], row["after"][head], atol=1e-10)
    for head in weights:
        np.testing.assert_allclose(weights[head], read(league / "policy.json")[head])
        counts[head + "_weight_change"] = float(np.linalg.norm(weights[head] - before[head]))
        assert counts[head + "_weight_change"] > 0.01
    if (R / "freeze.json").exists():
        raise ValueError("Existing freeze")
    shutil.copy2(league / "policy.json", R / "policy.json")
    shutil.copytree(league, R / "training")
    (R / "training-audit.json").write_text(json.dumps(counts, indent=2) + "\n")
    prior = read(ROOT / "policies/recipient_3v3_v1/freeze.json")["files"]
    assert all(digest(ROOT / k) == v for k, v in prior.items())
    paths = [ROOT / k for k in prior]
    paths += [
        R / "policy.json",
        R / "protocol.json",
        ROOT / "scripts/run_contest.py",
        ROOT / "scripts/audit_contest.py",
        ROOT / "scripts/freeze_contest.py",
        ROOT / "src/roboturk_core/roboturk/contest_outlet.py",
        ROOT / "src/roboturk_core/roboturk/contest_metrics.py",
        ROOT / "src/roboturk_ros/webots/controllers/roboturk_contest/roboturk_contest.py",
        ROOT / "src/roboturk_ros/webots/worlds/ssl_contest.wbt",
    ]
    (R / "freeze.json").write_text(
        json.dumps({str(p.relative_to(ROOT)): digest(p) for p in paths}, indent=2) + "\n"
    )
    print(counts)


if __name__ == "__main__":
    main()
