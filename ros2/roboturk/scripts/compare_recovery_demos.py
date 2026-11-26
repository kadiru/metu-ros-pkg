"""Compare every tracked state and decision across two independent native demo launches."""

import argparse
import hashlib
from pathlib import Path

import numpy as np
from audit_recovery_attack import panel
from run_recovery_attack import read, save


def main():
    p = argparse.ArgumentParser()
    p.add_argument("original", type=Path)
    p.add_argument("repeat", type=Path)
    p.add_argument("receipt", type=Path)
    a = p.parse_args()
    cfg = read(a.original / "launch.json")
    assert cfg == read(a.repeat / "launch.json") and cfg["mode"] == "demo"
    reference = panel(a.original)
    second = panel(a.repeat)
    games = []
    for i in range(cfg["cases"]):
        game = f"game-{i:03}.json"
        trace = f"trace-{i:03}.npz"
        left, right = read(a.original / game), read(a.repeat / game)
        with np.load(a.original / trace) as x, np.load(a.repeat / trace) as y:
            s, t = x["states"], y["states"]
            assert s.shape == t.shape
            games.append(
                dict(
                    seed=left["seed"],
                    samples=len(s),
                    max_state_delta=float(np.abs(s - t).max()),
                    same_options=left["options"] == right["options"],
                    same_events=left["events"] == right["events"],
                    same_conversions=left["conversions"] == right["conversions"],
                    repeat_state_sha256=hashlib.sha256(t.tobytes()).hexdigest(),
                )
            )
    identical = reference == second and all(
        g["max_state_delta"] == 0
        and g["same_options"]
        and g["same_events"]
        and g["same_conversions"]
        for g in games
    )
    save(
        a.receipt,
        dict(
            identical=identical,
            games=games,
            original_totals=reference["totals"],
            repeat_totals=second["totals"],
        ),
    )
    print("Identical games:", identical, [g["max_state_delta"] for g in games])
    return int(not identical)


if __name__ == "__main__":
    raise SystemExit(main())
