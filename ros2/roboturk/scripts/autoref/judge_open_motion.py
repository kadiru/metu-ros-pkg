"""Feed each native open-play segment to its own upstream TIGERs collision detector."""

import argparse
from contextlib import closing
from itertools import groupby
from pathlib import Path
import sys
from roboturk.model import World, RobotKey, Robot, Ball
from roboturk.ssl_vision import VisionOutput, practice_command
from run_fixture import service
from train_open_motion import ROOT, read_game, save
from open_motion_metrics import stats
from verify import load_events

CRASHES = {"bot_crash_unique", "bot_crash_drawn"}


def judge(path, distribution, classes, java, ci_port=21943):
    log, states = read_game(path)
    output = path / "official"
    output.mkdir()
    result = stats(log, states)
    frame_count = 0
    events = []
    keys = [RobotKey(*k) for k in log["keys"]]
    epoch = 1800000000.0
    for n, items in groupby(enumerate(log["frames"]), key=lambda it: it[1]["restart"]):
        base = output / f"segment-{n}"
        base.mkdir()
        cmd = [
            sys.executable,
            str(ROOT / "scripts/autoref/run_tigers.py"),
            str(base / "observer"),
            "--distribution",
            str(distribution),
            "--classes",
            str(classes),
            "--java",
            str(java),
            "--ci-port",
            str(ci_port),
            "--gc-port",
            str(ci_port + 4),
        ]
        log_path = base / "tigers.log"
        with service(cmd, ROOT, log_path, "SSL_READY"):
            with closing(VisionOutput(None, ("127.0.0.1", ci_port))) as vision:
                wire = practice_command(log["sign"])
                for i, frame in items:
                    state = states[i]
                    w = World(
                        i * log["dt"],
                        tuple(
                            Robot(k, *pose)
                            for k, pose in zip(keys, state[4:].reshape(10, 3), strict=True)
                        ),
                        Ball(*state[:4]),
                    )
                    stamp = epoch + i * log["dt"]
                    wire.packet_timestamp = round(stamp * 1e6)
                    vision.send(w, stamp, frame["ball_z"], wire)
                    frame_count += 1
        for stamp, kind, payload in load_events(log_path):
            events.append(
                dict(tick=round((stamp - epoch) / log["dt"]), segment=n, kind=kind, event=payload)
            )
    assert frame_count == len(states)
    result.update(
        seed=log["seed"],
        seconds=log["seconds"],
        score=log["score"],
        frames=frame_count,
        crashes=[e for e in events if e["kind"] in CRASHES],
        detector_events=events,
    )
    save(output / "result.json", result)
    return result


def paths(p):
    p.add_argument("--distribution", type=Path, default=ROOT.parent / "runtime/autoReferee")
    p.add_argument("--classes", type=Path, default=ROOT / "runs/autoref-v1")
    p.add_argument(
        "--java", type=Path, default=ROOT.parent / "runtime/java-autoref/Contents/Home/bin/java"
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("match", type=Path)
    paths(p)
    p.add_argument("--ci-port", type=int, default=21943)
    a = p.parse_args()
    judge(
        a.match.resolve(),
        a.distribution.resolve(),
        a.classes.resolve(),
        a.java.resolve(),
        a.ci_port,
    )
