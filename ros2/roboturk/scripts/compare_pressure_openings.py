"""Re-audit every physical sample and compare prior and trained teams on matched starts."""

from collections import Counter
from pathlib import Path
import argparse
import numpy as np

from train_pressure_receive import read, save, pressure_result


def compare(trained, control):
    paths = (trained, control)
    cfg, baseline = (read(d / "launch.json") for d in paths)
    assert cfg["schedule"] == baseline["schedule"] and cfg["press"] and baseline["press"]
    assert cfg["protocol"] == baseline["protocol"] and len(cfg["schedule"]) == 40
    results = []
    scored = []
    for i, (seed, _, _) in enumerate(cfg["schedule"]):
        traces = [np.load(d / f"trace-{i:03}.npz")["states"] for d in paths]
        assert np.array_equal(traces[0][0], traces[1][0]), seed
        rows = []
        for states, directory in zip(traces, paths, strict=True):
            game = read(directory / f"game-{i:03}.json")
            assert len(states) == 100 * cfg["seconds"] and game["seed"] == seed
            row = pressure_result(states, game, cfg["protocol"])
            assert row == game["opening"]
            rows.append(row)
        new, old = rows
        results.append(
            dict(
                seed=seed,
                trained=new["qualified"],
                prior=old["qualified"],
                trained_outcome=new["outcome"],
                prior_outcome=old["outcome"],
            )
        )
        if new["qualified"]:
            scored.append(new)
    return dict(
        same_first_frames=True,
        games=len(results),
        newly_converted=sum(r["trained"] and not r["prior"] for r in results),
        lost_conversions=sum(r["prior"] and not r["trained"] for r in results),
        trained_goals=len(scored),
        prior_goals=sum(r["prior"] for r in results),
        sides=dict(Counter(r["chain"]["team"] for r in scored)),
        median={
            k: float(np.median([r[k] for r in scored]))
            for k in ["gap_m", "approach_mps", "lead", "advance", "sole_samples"]
        }
        if scored
        else {},
        rows=results,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("trained", type=Path)
    p.add_argument("control", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    result = compare(a.trained, a.control)
    save(a.output, result)
    print({k: v for k, v in result.items() if k != "rows"})
