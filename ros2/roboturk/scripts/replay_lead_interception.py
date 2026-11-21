"""Re-record one of the original nine interceptions and its matched lead-pass goal."""

import argparse
from pathlib import Path
import numpy as np
from train_lead_outlet import R, PRIOR, ROOT, WORLD, read, save, sha
from train_recovery_transition import run_matches


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("historical", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument(
        "--webots",
        type=Path,
        default=ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots",
    )
    a = p.parse_args()
    old = PRIOR / "qualification-1"
    new = a.historical
    rows = read(new / "audit.json")["rows"]
    i = next(i for i, r in enumerate(rows) if r["qualified"])
    row = rows[i]
    seed = row["seed"]
    old_cfg = read(old / "launch.json")
    j = next(i for i, s in enumerate(old_cfg["schedule"]) if s[0] == seed)
    loss = next(r for r in read(R / "previous-interceptions.json") if r["seed"] == seed)
    a.output.mkdir(parents=True, exist_ok=False)
    proof = {}
    for name, src, slot, world in (
        ("previous", old, j, WORLD.with_name("ssl_blocked.wbt")),
        ("trained", new, i, WORLD),
    ):
        d = (a.output / name).resolve()
        d.mkdir()
        cfg = read(src / "launch.json") | dict(
            root=str(ROOT), cases=1, record=True, schedule=[[seed, 0, 0]]
        )
        assert cfg["webots_sha256"] == sha(a.webots)
        save(d / "launch.json", cfg)
        run_matches(d, cfg, a.webots, world, True)
        original = read(src / f"game-{slot:03}.json")
        repeat = read(d / "game-000.json")
        assert all(original[k] == repeat[k] for k in ("events", "passes", "options", "opening"))
        states = np.load(src / f"trace-{slot:03}.npz")["states"]
        assert np.array_equal(states, np.load(d / "trace-000.npz")["states"])
        proof[name] = dict(
            seed=seed, samples=len(states), max_state_delta=0, identical_events_options=True
        )
    kick = read(old / f"game-{j:03}.json")["opening"]["kick"]
    before = dict(outcome="Pass intercepted", elapsed_s=kick * 0.04 + loss["delay_s"])
    save(
        a.output / "replay.json",
        dict(stem="matched-lead", before=before, play=row["chain"], lead=row, proof=proof),
    )


if __name__ == "__main__":
    main()
