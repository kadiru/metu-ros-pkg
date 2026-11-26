"""Compare every frozen start with its matched previous-team run and verify lead passes."""

import argparse
from collections import Counter
from pathlib import Path
import numpy as np
from roboturk.lead_audit import lead_result
from run_recovery_attack import read, save


def compare(trained, baseline):
    cfg = read(trained / "launch.json")
    assert cfg["schedule"] == read(baseline / "launch.json")["schedule"]
    rows = []
    for i, (seed, _, _) in enumerate(cfg["schedule"]):
        games = [read(d / f"game-{i:03}.json") for d in (trained, baseline)]
        traces = [np.load(d / f"trace-{i:03}.npz")["states"] for d in (trained, baseline)]
        assert np.array_equal(traces[0][0], traces[1][0]), (
            f"First physical frame mismatch at {seed}"
        )
        for g, s in zip(games, traces, strict=True):
            assert seed == g["seed"] and len(s) == cfg["seconds"] * 100
            assert lead_result(s, g, cfg["protocol"]) == g["opening"]
        new, old = (g["opening"] for g in games)
        row = dict(
            seed=seed,
            old_goal=old["pass_goal"],
            new_goal=new["qualified"],
            old_intercepted=old["intercepted"],
            new_intercepted=new["intercepted"],
            old_outcome=old["outcome"],
            new_outcome=new["outcome"],
        )
        if new["qualified"]:
            start = 4 * new["kick"] - 1
            sample = traces[0][start]
            number = new["receiver"] + (3 if new["chain"]["team"] == "yellow" else 0)
            robot = sample[4:].reshape(6, 3)[number]
            mouth = robot[:2] + 0.115 * np.array([np.cos(robot[2]), np.sin(robot[2])])
            axis = mouth - sample[:2]
            axis /= np.linalg.norm(axis)
            delta = np.array(new["target"]) - mouth
            offset = abs(axis[0] * delta[1] - axis[1] * delta[0])
            row.update(
                lateral_target_offset=float(offset),
                team=new["chain"]["team"],
                receiver=new["receiver"],
                lead=new["lead"],
                advance=new["advance"],
                target_error=new["target_error"],
            )
        rows.append(row)
    totals = {
        k: sum(r[k] for r in rows)
        for k in ("old_goal", "new_goal", "old_intercepted", "new_intercepted")
    }
    leads = [r for r in rows if r["new_goal"]]
    metrics = (
        {k: float(np.median([r[k] for r in leads])) for k in ("lead", "advance", "target_error")}
        if leads
        else {}
    )
    return dict(
        games=len(rows),
        same_first_frames=True,
        totals=totals,
        qualifying_sides=dict(Counter(r["team"] for r in leads)),
        median_m=metrics,
        passed=len(rows) == 40 and totals["new_goal"] >= 24 and totals["new_intercepted"] <= 8,
        fewer_than_matched=totals["new_intercepted"] < totals["old_intercepted"],
        rows=rows,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("trained", type=Path)
    p.add_argument("baseline", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    result = compare(a.trained, a.baseline)
    save(a.output, result)
    print({k: v for k, v in result.items() if k != "rows"})
