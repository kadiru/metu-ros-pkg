"""Collect every touchline exit in a fixed sample of ordinary 5v5 games."""

import argparse
import gzip
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from run_fixture import ROOT, service
from verify import load_events, load_log


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("matches", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--distribution", type=Path, default=ROOT.parent / "runtime/autoReferee")
    p.add_argument("--classes", type=Path, default=ROOT / "runs/autoref-v1")
    p.add_argument(
        "--java", default=str(ROOT.parent / "runtime/java-autoref/Contents/Home/bin/java")
    )
    p.add_argument("--webots", type=Path)
    args = p.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    seeds = json.loads((ROOT / "policies/placement_v2/protocol.json").read_text())[
        "natural_match_seeds"
    ]
    cases = []
    for seed in seeds:
        folder = args.matches / str(seed)
        game = load_log(folder)
        with np.load(folder / "positions.npz") as f:
            states = f["states"]
        previous = -1
        for e in game["events"]:
            if e["event"] not in ("out", "goal"):
                continue
            i = e["tick"]
            x, y = states[i, :2]
            if e["event"] == "out" and 3 - abs(y) < 4.5 - abs(x):
                cases.append(
                    dict(
                        seed=seed,
                        sign=game["sign"],
                        tick=i,
                        state=states[i].tolist(),
                        before=states[max(previous + 1, i - 250) : i].tolist(),
                    )
                )
            previous = i
    if not cases:
        raise ValueError("No natural touchline exit in these matches")
    spec = output / "camera.json"
    spec.write_text(json.dumps(dict(mode="natural_camera", cases=cases)))
    cmd = [
        sys.executable,
        str(ROOT / "scripts/autoref/run_tigers.py"),
        str(output / "observer"),
        "--distribution",
        str(args.distribution.resolve()),
        "--classes",
        str(args.classes.resolve()),
        "--java",
        args.java,
        "--ci-port",
        "21923",
        "--gc-port",
        "21927",
    ]
    with service(cmd, ROOT, output / "tigers.log", "SSL_READY"):
        cmd = [
            sys.executable,
            str(ROOT / "scripts/run_five_roles.py"),
            str(output / "match"),
            "--placement-batch",
            str(spec),
            "--autoref-ci",
            "127.0.0.1:21923",
        ]
        if args.webots:
            cmd.extend(["--webots", str(args.webots)])
        subprocess.run(cmd, check=True)
    events = load_events(output / "tigers.log")
    compact = []
    for c, t in zip(cases, json.loads((output / "match/placement.json").read_text()), strict=True):
        calls = [
            e
            for ts, k, e in events
            if t["start"] <= ts < t["end"] and k == "ball_left_field_touch_line"
        ]
        if len(calls) != 1:
            raise ValueError(
                f"Expected one independent touchline call for {c['seed']}/{c['tick']}, got {calls}"
            )
        e = calls[0]["ballLeftFieldTouchLine"]
        x, y = e["location"]["x"], e["location"]["y"]
        compact.append(
            dict(
                seed=c["seed"],
                tick=c["tick"],
                sign=c["sign"],
                state=c["state"],
                target=[max(-4.3, min(4.3, x)), 2.8 if y > 0 else -2.8],
                team="blue" if e["byTeam"] == "YELLOW" else "yellow",
                exit=e,
            )
        )
    (output / "natural-starts.json").write_text(json.dumps(compact, indent=2) + "\n")
    (output / "camera.json.gz").write_bytes(gzip.compress(spec.read_bytes(), mtime=0))
    print(len(compact), "natural touchline exits classified")


if __name__ == "__main__":
    main()
