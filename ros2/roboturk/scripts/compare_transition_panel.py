"""Run the previous frozen team on the exact initial states of the completed new-team test."""

import argparse
from pathlib import Path
import numpy as np
from audit_recovery_attack import panel
from recovery_losses import analyze
from train_recovery_transition import R, ROOT, digest, read, save, run_matches


def main():
    p = argparse.ArgumentParser()
    p.add_argument("qualified", type=Path)
    p.add_argument("baseline", type=Path)
    p.add_argument("--version", type=int, default=1)
    p.add_argument(
        "--webots",
        type=Path,
        default=ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots",
    )
    a = p.parse_args()
    q = a.qualified.resolve()
    d = a.baseline.resolve()
    cfg = read(q / "launch.json")
    plan = read(R / "protocol.json")
    assert (cfg["first"], cfg["cases"], cfg["seconds"]) == (
        plan["qualification_first"] + 1000 * (a.version - 1),
        40,
        24,
    )
    tested = panel(q)
    assert tested["totals"]["first"] >= plan["target"]
    old = ROOT / "policies/recovery_attack_v2"
    for name, sha in read(old / "freeze.json").items():
        assert digest(ROOT / name) == sha, name
    launch = dict(
        root=str(ROOT),
        mode="trained",
        first=cfg["first"],
        cases=cfg["cases"],
        seconds=cfg["seconds"],
        model=str(old / "policy.json"),
        record=False,
        webots_sha256=digest(a.webots),
    )
    assert launch["webots_sha256"] == cfg["webots_sha256"]
    d.mkdir(parents=True, exist_ok=False)
    save(d / "launch.json", launch)
    run_matches(
        d, launch, a.webots, ROOT / "src/roboturk_ros/webots/worlds/ssl_shield_attack.wbt", True
    )
    before = panel(d)
    differences = []
    for i in range(cfg["cases"]):
        f = f"trace-{i:03}.npz"
        with np.load(d / f) as x, np.load(q / f) as y:
            delta = float(abs(x["states"][0] - y["states"][0]).max())
            assert delta < 1e-7
            differences.append(delta)
    losses = {"previous": analyze(d), "trained": analyze(q)}
    games = list(zip(before["games"], tested["games"], strict=True))
    evidence = dict(
        previous=before["totals"],
        trained=tested["totals"],
        max_initial_delta=max(differences),
        gained=sum(bool(y["first"] and not x["first"]) for x, y in games),
        lost=sum(bool(x["first"] and not y["first"]) for x, y in games),
        losses=losses,
    )
    save(d / "audit.json", before)
    save(d / "comparison.json", evidence)
    print({k: v for k, v in evidence.items() if k != "losses"})


if __name__ == "__main__":
    main()
