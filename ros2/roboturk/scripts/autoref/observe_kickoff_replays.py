"""Run the official TIGERs crash detector on each exact-start Webots trajectory."""

import argparse
from contextlib import closing
from hashlib import sha256
import json
from pathlib import Path
import time

import numpy as np

from roboturk.five_roles import CFG
from roboturk.model import World, Robot, RobotKey, Ball
from roboturk.ssl_vision import Packet, VisionOutput, practice_command
from run_fixture import ROOT, observer
from run_placement import CRASHES
from verify import load_events


def control_trace():
    """Synthetic head-on approach to exercise the same detector in kickoff preparation."""
    xy = np.array([(-3, i * 0.6) for i in range(-2, 3)] + [(3, i * 0.6) for i in range(-2, 3)])
    states = []
    for t in range(37):
        xy[0] = (-max(0.09, 1.90 - t * CFG.dt * 1.85), -2.5)
        xy[5] = (-xy[0, 0], -2.5)
        robots = np.column_stack((xy, np.zeros(10)))
        states.append(np.r_[np.zeros(4), robots.ravel()])
    return np.array(states)


def records(replays, cases):
    keys = [RobotKey(t, i) for t in ("blue", "yellow") for i in range(5)]
    yield "positive-control", control_trace(), keys, "blue", 1
    with np.load(replays / "kickoff-traces.npz") as traces:
        for c in cases:
            keys = [RobotKey(*key) for key in c["keys"]]
            for arm in ("shared", "teams"):
                name = f"{c['seed']}_{c['restart']}_{arm}"
                yield name, traces[name], keys, c["serving"], c["sign"]


def observe_trace(args, folder, states, keys, team, sign):
    folder.mkdir()
    packet = practice_command(sign)
    packet.stage = Packet.NORMAL_SECOND_HALF if sign < 0 else Packet.NORMAL_FIRST_HALF
    start = time.time()
    n_warm = round(1.6 / CFG.dt)
    ids_in_order = [(2 if k.team == "blue" else 1, k.id) for k in keys]
    counts = [0] * len(keys)
    with (
        observer(args, folder, ci_port=22913, gc_port=22907),
        closing(VisionOutput(None, ("127.0.0.1", 22913))) as vision,
    ):
        for i in range(-n_warm, len(states) + round(1.2 / CFG.dt)):
            stamp = start + (i + n_warm) * CFG.dt
            if i in (-n_warm, 0):
                packet.command = (
                    Packet.HALT if i < 0 else getattr(Packet, "PREPARE_KICKOFF_" + team.upper())
                )
                packet.command_counter += 1
                packet.command_timestamp = round(stamp * 1e6)
            packet.packet_timestamp = round(stamp * 1e6)
            s = states[max(0, min(i, len(states) - 1))]
            bots = tuple(
                Robot(k, *pose) for k, pose in zip(keys, s[4:].reshape(-1, 3), strict=True)
            )
            w = World(stamp, bots, Ball(*s[:4]), attack_sign=sign)
            tracked = vision.send(w, stamp, CFG.ball_radius, packet)
            if 0 <= i < len(states) and tracked is not None:
                assert abs(tracked.timestamp - stamp) < 0.001
                ids = {(bot.robot_id.team, bot.robot_id.id) for bot in tracked.robots}
                for idx, robot_id in enumerate(ids_in_order):
                    counts[idx] += robot_id in ids
    entry = dict(
        trace_sha256=sha256(states.tobytes()).hexdigest(),
        robot_samples=sum(counts),
        samples_by_robot=counts,
        expected=len(states) * len(keys),
        start_s=start + n_warm * CFG.dt,
        dt=CFG.dt,
    )
    (folder / "coverage.json").write_text(json.dumps(entry, indent=2) + "\n")


def read_observations(replays, cases, root):
    results = {}
    for name, states, _, _, _ in records(replays, cases):
        folder = root / name
        info = json.loads((folder / "coverage.json").read_text())
        assert info["trace_sha256"] == sha256(states.tobytes()).hexdigest()
        counts = info["samples_by_robot"]
        assert len(counts) == 10 and all(0 < n <= len(states) for n in counts)
        assert info["expected"] == len(states) * 10 and info["robot_samples"] == sum(counts)
        assert "SSL_READY PASSIVE" in (folder / "tigers.log").read_text()
        reports = [
            dict(seconds_from_start=round(stamp - info["start_s"], 3), packet=p)
            for stamp, _, p in load_events(folder / "tigers.log")
            if p["type"] in CRASHES
        ]
        results[name] = dict(**info, crashes=reports)
    assert results["positive-control"]["crashes"], (
        "Kickoff detector did not report the control collision"
    )
    return results


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("replays", type=Path)
    p.add_argument("cases", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--distribution", type=Path, default=ROOT.parent / "runtime/autoReferee")
    p.add_argument("--classes", type=Path, default=ROOT / "runs/autoref-v1")
    p.add_argument(
        "--java", type=Path, default=ROOT.parent / "runtime/java-autoref/Contents/Home/bin/java"
    )
    args = p.parse_args()
    cases = json.loads(args.cases.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    for name, states, keys, team, sign in records(args.replays, cases):
        print(name, flush=True)
        observe_trace(args, args.output / name, states, keys, team, sign)
    parsed = read_observations(args.replays, cases, args.output)
    print("Control crash reports", len(parsed["positive-control"]["crashes"]), flush=True)
