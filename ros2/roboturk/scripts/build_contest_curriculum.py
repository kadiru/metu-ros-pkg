"""Collect contested poses from ordinary development matches for native training."""

import argparse
import json
from pathlib import Path
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument("output", type=Path)
    p.add_argument("inputs", type=Path, nargs="+")
    a = p.parse_args()
    cases = []
    for folder in a.inputs:
        for f in sorted(folder.glob("game-*.json")):
            g = json.loads(f.read_text())
            assert 8900000 <= g["seed"] < 8920000
            states = np.load(f.with_name(f.name.replace("game", "trace").replace(".json", ".npz")))[
                "states"
            ]
            for e in g["contest_metrics"]["contests"]:
                i = e["start"] + 19
                cases.append(
                    dict(
                        seed=g["seed"],
                        sample=i,
                        source=folder.parent.name,
                        state=states[i].tolist(),
                    )
                )
    assert cases
    a.output.parent.mkdir(exist_ok=True, parents=True)
    a.output.write_text(json.dumps(cases, indent=2) + "\n")
    print(len(cases), "native contest poses")


if __name__ == "__main__":
    main()
