"""Judge each native tackle trial independently using the upstream passive TIGERs AutoRef."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import closing
import gzip
import json
from pathlib import Path
import sys

from judge_open_motion import CRASHES, paths
from roboturk.model import Ball, Robot, RobotKey, World
from roboturk.ssl_vision import VisionOutput, practice_command
from run_fixture import ROOT, service
from verify import load_events

DT = 0.04
EPOCH = 1800000000.0


def judge(args, row, output, worker):
    output.mkdir()
    port = args.port + 2 * worker
    cmd = [
        sys.executable,
        str(ROOT / "scripts/autoref/run_tigers.py"),
        str(output / "observer"),
        "--distribution",
        str(args.distribution.resolve()),
        "--classes",
        str(args.classes.resolve()),
        "--java",
        str(args.java.resolve()),
        "--ci-port",
        str(port),
        "--gc-port",
        str(port + 1),
    ]
    sign = row["initial"]["attack_sign"]
    keys = [RobotKey(**b["key"]) for b in row["initial"]["robots"]]
    wire = practice_command(sign)
    trace = row["trace"]
    if not trace:
        raise ValueError("Record native tackle trials with --motion-audit")
    log = output / "observer.log"
    with service(cmd, ROOT, log, "SSL_READY"):
        with closing(VisionOutput(None, ("127.0.0.1", port))) as vision:
            # Establish tracks before the first live motion. A separate process
            # prevents ball/player resets from becoming apparent collisions.
            for tick in range(-12, len(trace)):
                sample = trace[max(0, tick)]
                robots = tuple(
                    Robot(key, *sample[4 + 3 * i : 7 + 3 * i]) for i, key in enumerate(keys)
                )
                world = World(tick * DT, robots, Ball(*sample[:4]), attack_sign=sign)
                stamp = EPOCH + tick * DT
                wire.packet_timestamp = round(stamp * 1e6)
                vision.send(world, stamp, sample[-1], wire)
    events = [
        dict(tick=round((stamp - EPOCH) / DT), kind=kind, event=event)
        for stamp, kind, event in load_events(log)
    ]
    return dict(
        seed=row["seed"],
        frames=len(trace),
        crashes=[e for e in events if e["kind"] in CRASHES and e["tick"] >= 0],
        all_events=events,
    )


def run(args):
    with gzip.open(args.trials, "rt") as stream:
        rows = [json.loads(line) for line in stream]
    if not rows or len({row["seed"] for row in rows}) != len(rows):
        raise ValueError("The fixture must contain distinct completed trials")
    args.output.mkdir(parents=True)
    with ThreadPoolExecutor(args.workers) as pool:
        pending = [
            pool.submit(judge, args, row, args.output / str(row["seed"]), i % args.workers)
            for i, row in enumerate(rows[: args.workers])
        ]
        results = []
        # Reuse a port only after its previous worker's process has shut down.
        for i in range(len(rows)):
            worker = i % args.workers
            results.append(pending[worker].result())
            upcoming = i + args.workers
            if upcoming < len(rows):
                row = rows[upcoming]
                pending[worker] = pool.submit(
                    judge, args, row, args.output / str(row["seed"]), worker
                )
    (args.output / "result.json").write_text(json.dumps(results, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trials", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--workers", type=int, choices=(1, 2, 3, 4), default=3)
    parser.add_argument("--port", type=int, default=22300)
    paths(parser)
    run(parser.parse_args())
