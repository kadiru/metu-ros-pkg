"""Replay the native return updates and freeze the selected 3v3 recovery policy."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np

from fit_contest_returns import fit
from roboturk.jaw_recovery import ReturnLearner
from run_contest import ROOT
from run_outlet_league import V1, V2, read, save, verify


def main():
    p = argparse.ArgumentParser()
    p.add_argument("jaw_league", type=Path)
    a = p.parse_args()
    verify(V1 / "freeze.json")
    verify(V1 / "isolation-freeze.json")
    m, _ = fit([V1 / "training", V2 / "league-two"], read(V1 / "policy.json"))
    assert m == read(V2 / "return-fit/policy.json")
    cfg = read(a.jaw_league / "launch.json")
    assert cfg["mode"] == "train_jaw" and cfg["matches"] == 256
    learner = ReturnLearner()
    obs = dict(episodes=0, decisions=0, recoveries=0, completions=0)
    by_group = {}
    for i in range(cfg["matches"]):
        g = read(a.jaw_league / f"game-{i:03}.json")
        assert g["seed"] == cfg["first"] + i and g["challenger"] == ("yellow" if i % 2 else "blue")
        reward = learner.update(g)
        assert reward == g["reward"]
        group = g["opponent"] if isinstance(g["opponent"], str) else "historical"
        bucket = by_group.setdefault(group, [0, 0, 0])
        bucket[0] += 1
        bucket[1] += reward > 0
        bucket[2] += reward > 1
        obs["episodes"] += 1
        obs["recoveries"] += reward > 0
        obs["completions"] += reward > 1
        obs["decisions"] += sum(
            v["head"] == "jaw" and v["team"] == g["challenger"] for v in g["learned_actions"]
        )
    m["jaw"] = learner.policy()
    native = read(a.jaw_league / "policy.json")
    np.testing.assert_allclose(m["jaw"]["q"], native["jaw"]["q"], rtol=1e-12, atol=1e-12)
    assert m.keys() == native.keys() and all(m[k] == native[k] for k in m if k != "jaw")
    assert obs["completions"] and np.std(m["jaw"]["q"]) > 0.01
    assert not (V2 / "freeze.json").exists()
    if a.jaw_league.resolve() != V2 / "jaw-training":
        shutil.copytree(a.jaw_league, V2 / "jaw-training")
    shutil.copyfile(a.jaw_league / "policy.json", V2 / "policy.json")
    save(V2 / "training-audit.json", dict(**obs, by_group_episodes_recoveries_completions=by_group))
    paths = [
        V1 / "freeze.json",
        V1 / "isolation-freeze.json",
        V2 / "protocol.json",
        V2 / "policy.json",
        V2 / "training-poses.json",
        ROOT / "scripts/run_outlet_league.py",
        ROOT / "scripts/fit_contest_returns.py",
        ROOT / "scripts/freeze_outlet_league.py",
        ROOT / "scripts/build_contest_curriculum.py",
        ROOT / "src/roboturk_core/roboturk/jaw_recovery.py",
        ROOT / "src/roboturk_ros/webots/worlds/ssl_outlet_league.wbt",
        ROOT
        / "src/roboturk_ros/webots/controllers/roboturk_outlet_league/roboturk_outlet_league.py",
    ]
    save(
        V2 / "freeze.json",
        {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
    )
    print(json.dumps(obs))


if __name__ == "__main__":
    main()
