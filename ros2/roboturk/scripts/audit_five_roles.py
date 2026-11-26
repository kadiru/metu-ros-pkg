"""Verify ten assigned robots on each control frame and locate open-play role changes."""

import argparse
from collections import Counter
import csv
import gzip
import json
from pathlib import Path
import numpy as np


def load_match(folder):
    log = json.loads(gzip.decompress((folder / "roles.json.gz").read_bytes()))
    return log, np.load(folder / "positions.npz")["states"]


def audit(folder):
    g, s = load_match(folder)
    keys = g["keys"]
    frames = g["frames"]
    n = len(frames)
    dt = g["dt"]
    assert keys == [[t, i] for t in ("blue", "yellow") for i in range(5)]
    assert s.shape == (n, 34) and n == round(g["seconds"] / dt) and np.isfinite(s).all()
    xy = s[:, 4:].reshape(n, 10, 3)
    delta = s[:, None, :2] - xy[:, :, :2]
    co, si = np.cos(xy[:, :, 2]), np.sin(xy[:, :, 2])
    mouth = (abs(delta[:, :, 0] * co + delta[:, :, 1] * si - 0.115) < 0.035) & (
        abs(delta[:, :, 1] * co - delta[:, :, 0] * si) < 0.045
    )
    transitions = []
    changes = Counter()
    holders = Counter()
    last = None
    for i, f in enumerate(frames):
        assert len(f["roles"]) == 10 and (i == 0 or f["clock"] > frames[i - 1]["clock"])
        for k in (0, 5):
            roles = f["roles"][k : k + 5]
            assert roles[0] == "Keeper" and len(set(roles)) == 5
        if i and f["restart"] == frames[i - 1]["restart"]:
            for k, (a, b) in enumerate(zip(frames[i - 1]["roles"], f["roles"])):
                if a != b:
                    changes[k] += 1
        else:
            last = None
        k = f["control"]
        if k is None:
            continue
        holders[k] += 1
        if k != last:
            assert (
                i >= 2
                and mouth[i - 2 : i + 1, k].all()
                and (mouth[i - 2 : i + 1].sum(axis=1) == 1).all()
            )
            if last is not None:
                transitions.append(
                    dict(
                        tick=i,
                        time=round(i * dt, 2),
                        before=last,
                        after=k,
                        event="turnover" if k // 5 != last // 5 else "handoff",
                        restart=f["restart"],
                    )
                )
            last = k
    passes = [e for e in g["events"] if e["event"] == "pass"]
    received = []
    for e in passes:
        for i in range(e["tick"] + 1, min(n, e["tick"] + 50)):
            f = frames[i]
            if f["restart"] != frames[e["tick"]]["restart"]:
                break
            if f["control"] is None or f["control"] == e["actor"]:
                continue
            if f["control"] == e["receiver"] and (
                np.linalg.norm(s[i, :2] - s[e["tick"], :2]) >= 0.55
            ):
                received.append(
                    dict(kick=e["tick"], receive=i, passer=e["actor"], receiver=e["receiver"])
                )
            break
    # Favor an unbroken passage with both a team turnover and a completed pass.
    span = min(n, round(12 / dt))

    def value(start):
        end = start + span
        turns = sum(
            start + 12 <= v["tick"] < end - 16 and v["event"] == "turnover" for v in transitions
        )
        catches = sum(start + 5 <= v["kick"] < v["receive"] < end - 12 for v in received)
        stable = frames[end - 1]["restart"] == frames[start]["restart"]
        return (
            stable and bool(turns) and bool(catches),
            stable and bool(turns),
            min(catches, 4),
            any(start + 8 <= e["tick"] < end - 5 and e["event"] == "shot" for e in g["events"]),
            min(turns, 2),
            -start,
        )

    start = max(range(n - span + 1), key=value)
    return dict(
        seed=g["seed"],
        seconds=g["seconds"],
        score=g["score"],
        control_frames=n,
        assignments_checked=10 * n,
        open_play_role_changes=[changes[i] for i in range(10)],
        control_frames_by_player=[holders[i] for i in range(10)],
        turnovers=sum(e["event"] == "turnover" for e in transitions),
        passes=len(passes),
        confirmed_passes=received,
        control_changes=transitions,
        excerpt=[start, start + span],
        events=dict(Counter(e["event"] for e in g["events"])),
    )


def write_changes(folder):
    game, _ = load_match(folder)
    with (folder / "role-changes.csv").open("w", newline="") as out:
        ledger = csv.writer(out)
        ledger.writerow(
            ["match_seconds", "team", "jersey", "old_role", "new_role", "phase", "restart"]
        )
        previous = None
        for i, f in enumerate(game["frames"]):
            for k, role in enumerate(f["roles"]):
                old = previous["roles"][k] if previous else ""
                if role != old or (previous and previous["restart"] != f["restart"]):
                    team, jersey = game["keys"][k]
                    ledger.writerow(
                        [
                            f"{i * game['dt']:.2f}",
                            team,
                            jersey,
                            old,
                            role,
                            f["phases"][k // 5],
                            f["restart"],
                        ]
                    )
            previous = f


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("directory", type=Path)
    p.add_argument("--replay", type=Path)
    a = p.parse_args()
    result = audit(a.directory)
    write_changes(a.directory)
    if a.replay:
        g, s = load_match(a.directory)
        other, t = load_match(a.replay)
        assert g == other and np.array_equal(s, t), "The recorded run diverged"
        result["exact_recorded_replay"] = True
    (a.directory / "audit.json").write_text(json.dumps(result, indent=2) + "\n")
    print(
        json.dumps(
            {k: v for k, v in result.items() if k not in ("control_changes", "confirmed_passes")}
        )
    )
