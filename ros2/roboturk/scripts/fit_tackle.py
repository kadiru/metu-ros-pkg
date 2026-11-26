"""Select a native Webots tackle profile by clean takeaways, contacts, then time."""

import argparse
import gzip
import json
from pathlib import Path
from statistics import median


def rows(path):
    with gzip.open(path, "rt") as stream:
        return [json.loads(line) for line in stream]


def starts(trials):
    return [(row["seed"], row["initial"]) for row in trials]


def fit(folders, output):
    results = []
    expected = None
    for folder in folders:
        config = json.loads((folder / "config.json").read_text())
        for name, params in config["tackle_sweep"].items():
            trials = rows(folder / name / "trials.jsonl.gz")
            if len(trials) != config["tackle_trials"] or starts(trials) != (
                expected or starts(trials)
            ):
                raise ValueError("Training candidates need the same complete native starts")
            expected = starts(trials)
            clean = [r["capture_s"] for r in trials if r["clean_takeaway"]]
            results.append(
                dict(
                    name=name,
                    weights=params,
                    starts=len(trials),
                    clean_takeaways=len(clean),
                    contacts=sum(r["contact"] for r in trials),
                    median_clean_capture_s=median(clean) if clean else None,
                )
            )
    best = min(
        results,
        key=lambda r: (
            -r["clean_takeaways"],
            r["contacts"],
            r["median_clean_capture_s"]
            if r["median_clean_capture_s"] is not None
            else float("inf"),
            r["name"],
        ),
    )
    output.mkdir(parents=True, exist_ok=True)
    (output / "actor.json").write_text(json.dumps(best["weights"], indent=2) + "\n")
    (output / "fit.json").write_text(
        json.dumps(
            dict(
                selected=best["name"], candidates=results, training_seeds=[s for s, _ in expected]
            ),
            indent=2,
        )
        + "\n"
    )
    print(json.dumps(best, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("training_runs", type=Path, nargs="+")
    args = parser.parse_args()
    fit(args.training_runs, args.output)
