"""Record the same historical opening twice, and verify its physical trajectories."""

import argparse
from pathlib import Path
import numpy as np
from train_recovery_touch import ROOT, WEBOTS, WORLD, read, save, sha, run_matches


def record(src, slot, dst, exe):
    cfg = read(src / "launch.json")
    expected = read(src / f"game-{slot:03}.json")
    cfg.update(root=str(ROOT), record=True, study=False, cases=1, schedule=[cfg["schedule"][slot]])
    assert cfg["webots_sha256"] == sha(exe)
    dst.mkdir(parents=True, exist_ok=False)
    save(dst / "launch.json", cfg)
    world = ROOT / cfg["world"] if "world" in cfg else WORLD
    if world not in (WORLD, WORLD.with_name("ssl_touch_planner.wbt")):
        raise ValueError("Unexpected recording world")
    run_matches(dst, cfg, exe, world, True)
    game = read(dst / "game-000.json")
    assert game == expected
    states = np.load(dst / "trace-000.npz")["states"]
    assert np.array_equal(states, np.load(src / f"trace-{slot:03}.npz")["states"])
    return states, game


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("previous", type=Path)
    p.add_argument("trained", type=Path)
    p.add_argument("pair", type=Path)
    p.add_argument("--webots", type=Path, default=WEBOTS)
    a = p.parse_args()
    old, new = [read(d / "audit.json")["rows"] for d in [a.previous, a.trained]]
    assert [r["seed"] for r in old] == [r["seed"] for r in new]
    slots = [
        i
        for i, (x, y) in enumerate(zip(old, new, strict=True))
        if x["prepass_loss"] and y["qualified"] and x["chain"]["winner"] == y["chain"]["winner"]
    ]
    # Select the fastest documented original loss among repaired openings.
    i = min(slots, key=lambda j: old[j]["first_break"] - old[j]["chain"]["recovery"])
    a.pair.mkdir(parents=True, exist_ok=False)
    left, g = record(a.previous, i, (a.pair / "previous").resolve(), a.webots)
    right, h = record(a.trained, i, (a.pair / "trained").resolve(), a.webots)
    assert np.array_equal(left[0], right[0])
    r = g["opening"]
    after = h["opening"]
    sample = r["first_break"]
    save(
        a.pair / "replay.json",
        dict(
            stem="first-touch-comparison",
            seed=r["seed"],
            before=dict(outcome="Lost ball before pass", elapsed_s=(sample + 1) / 100),
            original=r,
            trained=after,
            play=after["chain"],
            first_touches=h["first_touches"],
            first_different_sample=int(np.flatnonzero((left != right).any(1))[0]),
            same_first_frame=True,
            reproduced_samples_per_side=len(left),
            max_replay_delta=0,
        ),
    )


if __name__ == "__main__":
    main()
