"""Compare independent TIGERs protobuf events to native archived match evidence."""

import argparse
import base64
import gzip
import json
from pathlib import Path

import numpy as np
from google.protobuf.json_format import MessageToDict
from roboturk.ssl_vision import GameEvent

ROOT = Path(__file__).resolve().parents[2]
BOUNDARIES = {
    "possible_goal",
    "ball_left_field_touch_line",
    "ball_left_field_goal_line",
    "aimless_kick",
}


def load_events(path):
    rows = []
    for line in path.read_text().splitlines():
        if line.startswith("SSL_EVENT "):
            _, ns, payload = line.split()
            e = GameEvent.FromString(base64.b64decode(payload, validate=True))
            rows.append((int(ns) / 1e9, e.WhichOneof("event"), MessageToDict(e)))
    return rows


def load_log(directory):
    with gzip.open(directory / "roles.json.gz", "rt") as f:
        return json.load(f)


def verify_passive(path):
    reports = []
    for seed in (20300000, 20300001, 20300002):
        folder = path / str(seed)
        run = folder / "match"
        old = ROOT / f"media/five-roles/matches/{seed}"
        current, archived = load_log(run), load_log(old)
        a, b = (np.load(d / "positions.npz")["states"] for d in (run, old))
        if not np.array_equal(a, b) or current["events"] != archived["events"]:
            raise ValueError(f"Archive trajectory or events differ for {seed}")
        if [
            {k: v for k, v in f.items() if k != "vision_time"} for f in current["frames"]
        ] != archived["frames"]:
            raise ValueError(f"Role frames differ for {seed}")
        expected = [e for e in archived["events"] if e["event"] in ("goal", "out")]
        all_calls = load_events(folder / "tigers.log")
        calls = [c for c in all_calls if c[1] in BOUNDARIES]
        findings = []
        for e, (stamp, name, packet) in zip(expected, calls, strict=True):
            i = e["tick"]
            delay = stamp - current["frames"][i]["vision_time"]
            detail = next(v for v in packet.values() if isinstance(v, dict))
            team = detail.get("byTeam", "").lower()
            # The archived event has no line classification. Use its measured
            # pre-crossing position to identify the nearest field boundary.
            x, y = a[i, :2]
            touch = 3 - abs(y) < 4.5 - abs(x)
            wanted = (
                {"possible_goal"}
                if e["event"] == "goal"
                else {"ball_left_field_touch_line"}
                if touch
                else {"ball_left_field_goal_line", "aimless_kick"}
            )
            if (
                name not in wanted
                or not 0 <= delay <= 1.04
                or (e["event"] == "goal" and e["team"] != team)
            ):
                raise ValueError(f"{seed} tick {i}: {e} vs {packet}, latency {delay}")
            findings.append(
                dict(
                    tick=i,
                    match_seconds=round(i * current["dt"], 2),
                    expected=e,
                    observed=name,
                    detection_delay_s=round(delay, 3),
                    protobuf=packet,
                )
            )
        reports.append(
            dict(
                seed=seed,
                exact_frames=len(a),
                boundaries=findings,
                other_events=[p for _, k, p in all_calls if k not in BOUNDARIES],
            )
        )
    return reports


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(verify_passive(args.runs), indent=2) + "\n")


def verify_active(folder):
    """Require the same scoring sequence at the observer, official and Webots."""
    protocol = [
        json.loads(row)
        for row in (folder / "gc/config/state-store.json.stream").read_text().splitlines()
    ]
    changes = [row["change"] for row in protocol]
    if any("updateTeamStateChange" in c and "goals" in c["updateTeamStateChange"] for c in changes):
        raise ValueError("Score was edited")
    additions = [c["addGameEventChange"]["gameEvent"] for c in changes if "addGameEventChange" in c]
    proposed = [
        e["possibleGoal"]["byTeam"].lower()
        for e in additions
        if e["type"] == "POSSIBLE_GOAL" and "TIGERs AutoRef" in e["origin"]
    ]
    approved = [
        e["goal"]["byTeam"].lower()
        for e in additions
        if e["type"] == "GOAL" and e["goal"].get("message") == "Valid and auto approved"
    ]
    calls = [
        p["possibleGoal"]["byTeam"].lower()
        for _, k, p in load_events(folder / "tigers.log")
        if k == "possible_goal"
    ]
    frames = load_log(folder / "match")["frames"]
    states = np.load(folder / "match/positions.npz")["states"]
    scores = [tuple(f["score"]) for f in frames]
    changed = [i for i in range(1, len(frames)) if scores[i] != scores[i - 1]]
    scorers = []
    sequences = []
    for pos, i in enumerate(changed):
        diff = tuple(a - b for a, b in zip(scores[i], scores[i - 1], strict=True))
        if diff not in ((1, 0), (0, 1)):
            raise ValueError("Expected a single goal")
        scoring = "blue" if diff == (1, 0) else "yellow"
        opponent = "yellow" if scoring == "blue" else "blue"
        stop = changed[pos + 1] if pos + 1 < len(changed) else len(frames)
        prep = next(
            j
            for j in range(i, stop)
            if frames[j]["ssl_command"] == "PREPARE_KICKOFF_" + opponent.upper()
        )
        start = next(
            j
            for j in range(prep, stop)
            if frames[j]["ssl_command"] == "NORMAL_START" and frames[j]["restricted_to"] == opponent
        )
        open_play = next(
            j
            for j in range(start, stop)
            if frames[j]["ssl_command"] == "NORMAL_START"
            and frames[j]["mode"] == "running"
            and not frames[j]["restricted_to"]
        )
        moved = float(np.linalg.norm(states[open_play, :2] - states[start, :2]))
        if moved < 0.05:
            raise ValueError("Opponent did not physically put the ball in play")
        scorers.append(scoring)
        sequences.append(
            dict(
                scorer=scoring,
                score=scores[i],
                score_tick=i,
                kickoff=opponent,
                prepare_tick=prep,
                restart_tick=start,
                open_play_tick=open_play,
                restart_ball_travel_m=round(moved, 4),
            )
        )
    if not scorers or calls != proposed or proposed != approved or approved != scorers:
        raise ValueError(f"Goal mismatch: {calls}, {proposed}, {approved}, {scorers}")
    return dict(
        seed=load_log(folder / "match")["seed"],
        dt=0.04,
        frames=len(frames),
        decisions=sequences,
        official_events=additions,
        observer_events=[p for _, _, p in load_events(folder / "tigers.log")],
    )
