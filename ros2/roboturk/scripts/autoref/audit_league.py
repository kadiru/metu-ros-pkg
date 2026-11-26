"""Verify paired match starts, native receptions and scoring through the final whistle."""

import argparse
from collections import Counter, defaultdict, deque
import json
from itertools import groupby
from pathlib import Path

import numpy as np
from google.protobuf.json_format import ParseDict
from roboturk.ssl_vision import GameEvent

from open_motion_metrics import stats
from run_league import ARMS, SEEDS
from run_placement import CRASHES
from train_open_motion import read_game, save
from verify import load_events

HALVES = ("NORMAL_FIRST_HALF", "NORMAL_SECOND_HALF")
STAGES = (
    HALVES[0] + "_PRE",
    HALVES[0],
    "NORMAL_HALF_TIME",
    HALVES[1] + "_PRE",
    HALVES[1],
    "POST_GAME",
)
TEAMS = ("BLUE", "YELLOW")


def payload(event):
    message = ParseDict(event, GameEvent())
    return getattr(message, message.WhichOneof("event")).SerializeToString(deterministic=True)


def verify_halves(log):
    frames = log["frames"]
    stages = list(dict.fromkeys(f["stage"] for f in frames if f["stage"]))
    assert tuple(stages) == STAGES and frames[-1]["stage"] == "POST_GAME"
    starts = [i for i, f in enumerate(frames) if f["first_action"]]
    assert len(starts) == 1 and all(f["stage"] for f in frames[starts[0] :])
    opening = starts[0]
    half_info = []
    for idx, half in enumerate(HALVES):
        period = [f for f in frames if f["stage"] == half]
        running = [f for f in period if f["mode"] == "running"]
        assert running and min(f["stage_seconds"] for f in period) <= 0
        sign = 1 if idx == 0 else -1
        assert all(f["attack_sign"] == sign for f in running)
        first = running[0]
        assert first["ssl_command"] == "NORMAL_START"
        assert first["restricted_to"] == TEAMS[(log["seed"] + idx) % 2].lower()
        half_info.append(
            dict(
                stage=half,
                kickoff=first["restricted_to"],
                blue_attack_sign=sign,
                wire_start_s=first["stage_seconds"],
                wire_end_s=period[-1]["stage_seconds"],
                running_native_s=round(len(running) * log["dt"], 2),
            )
        )

    return opening, half_info


def official_events(folder):
    rows = [
        json.loads(line)
        for line in (folder / "gc/config/state-store.json.stream").read_text().splitlines()
    ]
    changes = [r["change"] for r in rows]
    assert all("goals" not in c.get("updateTeamStateChange", {}) for c in changes)
    official = [c["addGameEventChange"]["gameEvent"] for c in changes if "addGameEventChange" in c]
    assert len({e["id"] for e in official}) == len(official)
    raw = load_events(folder / "tigers.log")
    emitted = Counter((p["type"], payload(p)) for _, _, p in raw)
    proposals = [
        c["addProposalChange"]["proposal"]["gameEvent"] for c in changes if "addProposalChange" in c
    ]
    origin = {}
    for e in proposals:
        if "TIGERs AutoRef" in e.get("origin", []):
            key = (e["type"], payload(e))
            assert emitted[key] > 0, key
            emitted[key] -= 1
            origin[e["type"], e["createdTimestamp"]] = e
    for e in official:
        if "TIGERs AutoRef" in e.get("origin", []):
            proposal = origin[e["type"], e["createdTimestamp"]]
            if e["type"] in CRASHES or e["type"] == "POSSIBLE_GOAL":
                assert payload(proposal) == payload(e)

    return raw, official


def verify_goals(frames, states, official, *, require_native=True):
    scoring = [e for e in official if e["type"] == "GOAL"]
    assert all(e["goal"].get("message") == "Valid and auto approved" for e in scoring)
    pending = []
    for event in official:
        if event["type"] == "POSSIBLE_GOAL" and "TIGERs AutoRef" in event["origin"]:
            pending.append(event["possibleGoal"])
        elif event["type"] == "GOAL":
            # The GC copies the original shot evidence into its goal and appends a verdict.
            proposal = dict(event["goal"], message="")
            assert proposal in pending
            pending.remove(proposal)
    wire_goals = []
    for i in range(1, len(frames)):
        f, prev = frames[i], frames[i - 1]
        delta = tuple(a - b for a, b in zip(f["score"], prev["score"], strict=True))
        if delta == (0, 0):
            continue
        assert delta in ((0, 1), (1, 0))
        team = TEAMS[delta.index(1)]
        direction = f["attack_sign"] * (1 if team == "BLUE" else -1)
        crossing = [
            j
            for j in range(i - 1, -1, -1)
            if frames[j]["restart"] == f["restart"]
            and frames[j]["stage"] == f["stage"]
            and states[j, 0] * direction > 4.5215
            and abs(states[j, 1]) < 0.4785
            and frames[j]["ball_z"] < 0.16
        ]
        if require_native:
            assert crossing, (team, i)
        goal = dict(
            team=team,
            score_tick=i,
            crossing_tick=min(crossing) if crossing else None,
            score=f["score"],
        )
        if not crossing:
            # Preserve the official decision and expose the sampled trajectory on marginal shots.
            center_crossings = []
            for j in range(1, i + 1):
                if any(
                    frames[k]["restart"] != f["restart"] or frames[k]["stage"] != f["stage"]
                    for k in (j - 1, j)
                ):
                    continue
                x0, x1 = states[j - 1 : j + 1, 0] * direction
                if x0 < 4.5 <= x1:
                    part = (4.5 - x0) / (x1 - x0)
                    center_crossings.append(
                        dict(
                            before_tick=j - 1,
                            after_tick=j,
                            interpolated_y_m=float(
                                states[j - 1, 1] + part * (states[j, 1] - states[j - 1, 1])
                            ),
                        )
                    )
            goal["center_line_crossings"] = center_crossings
        wire_goals.append(goal)
    assert [g["team"] for g in wire_goals] == [e["goal"]["byTeam"] for e in scoring]
    assert frames[-1]["score"] == [sum(g["team"] == t for g in wire_goals) for t in TEAMS]

    return wire_goals


def verify_receptions(log, states):
    frames = log["frames"]
    kicks = [dict(e, event="pass") if e["event"] == "restart_pass" else e for e in log["events"]]
    results = stats(dict(log, events=kicks), states)
    deadballs = {e["tick"] for e in log["events"] if e["event"] == "restart_pass"}
    passes = [
        dict(p, restart_pass=p["kick"] in deadballs)
        for p in results["passes"]
        if all(
            f["mode"] == "running" and f["counter"] == frames[p["kick"]]["counter"]
            for f in frames[p["kick"] : p["receive"] + 1]
        )
    ]
    return passes, results["attempts"]


def locate_crashes(log, states, raw, official):
    frames = log["frames"]
    times = np.array([f["vision_time"] for f in frames])
    xy = states[:, 4:].reshape(-1, 10, 3)[:, :, :2]
    formations = [
        i for i, f in enumerate(frames) if i == 0 or f["restart"] != frames[i - 1]["restart"]
    ]
    accepted_calls = defaultdict(deque)
    for event in official:
        if event["type"] in CRASHES:
            accepted_calls[payload(event)].append(event["id"])
    crashes = []
    for stamp, _, p in raw:
        if p["type"] in CRASHES:
            tick = int(np.argmin(abs(times - stamp)))
            if p["type"] == "BOT_CRASH_DRAWN":
                details = p["botCrashDrawn"]
                blue, yellow = details["botBlue"], details["botYellow"]
            else:
                details = p["botCrashUnique"]
                blue, yellow = details["violator"], details["victim"]
                if details["byTeam"] == "YELLOW":
                    blue, yellow = yellow, blue
            indices = [
                log["keys"].index([team, number])
                for team, number in (("blue", blue), ("yellow", yellow))
            ]
            window = xy[max(0, tick - round(1 / log["dt"])) : tick + 1]
            nearest = float(
                np.linalg.norm(window[:, indices[0]] - window[:, indices[1]], axis=1).min()
            )
            reset = max(j for j in formations if j <= tick)
            ids = accepted_calls[payload(p)]
            crashes.append(
                dict(
                    tick=tick,
                    official_id=ids.popleft() if ids else None,
                    blue=blue,
                    yellow=yellow,
                    command=frames[tick]["ssl_command"],
                    event=p,
                    since_formation_s=round((tick - reset) * log["dt"], 2),
                    nearest_centers_previous_second_m=round(nearest, 4),
                )
            )
    assert not any(accepted_calls.values())
    return crashes


def audit(folder, *, require_native_goals=True):
    log, states = read_game(folder / "match")
    frames = log["frames"]
    opening, half_info = verify_halves(log)
    raw, official = official_events(folder)
    goals = verify_goals(frames, states, official, require_native=require_native_goals)
    passes, attempts = verify_receptions(log, states)
    crashes = locate_crashes(log, states, raw, official)
    report = dict(
        seed=log["seed"],
        final_score=frames[-1]["score"],
        official_halves=half_info,
        kickoff_prepare_native_seconds=[
            round(sum(1 for _ in group) * log["dt"], 2)
            for command, group in groupby(frames, lambda f: f["ssl_command"])
            if command.startswith("PREPARE_KICKOFF_")
        ],
        native_seconds_by_mode={
            mode: round(n * log["dt"], 2) for mode, n in Counter(f["mode"] for f in frames).items()
        },
        simulated_seconds=log["seconds"],
        opening_tick=opening,
        goals=goals,
        completed=passes,
        attempts=attempts,
        observed_crashes=crashes,
        accepted_crashes=[e for e in official if e["type"] in CRASHES],
        accepted_events=dict(Counter(e["type"] for e in official)),
        detector_events=dict(Counter(p["type"] for _, _, p in raw)),
    )
    return report, states[opening], frames[opening]["opening_velocities"]


def summarize(root, seeds=SEEDS):
    pairs = []
    totals = {arm: Counter() for arm in ARMS}
    for seed in seeds:
        results = [audit(root / str(seed) / arm) for arm in ARMS]
        assert np.array_equal(results[0][1], results[1][1]) and np.array_equal(
            results[0][2], results[1][2]
        )
        pair = dict(seed=seed, identical_opening=True)
        for arm, (r, _, _) in zip(ARMS, results, strict=True):
            pair[arm] = r
            totals[arm].update(
                dict(
                    observed_crashes=len(r["observed_crashes"]),
                    accepted_crashes=len(r["accepted_crashes"]),
                    calls_with_centers_within_30cm=sum(
                        c["nearest_centers_previous_second_m"] <= 0.30
                        for c in r["observed_crashes"]
                    ),
                    completed_passes=len(r["completed"]),
                    restart_completions=sum(p["restart_pass"] for p in r["completed"]),
                    pass_attempts=r["attempts"],
                    goals=len(r["goals"]),
                    native_play_seconds=sum(h["running_native_s"] for h in r["official_halves"]),
                )
            )
        pairs.append(pair)
    for arm in ARMS:
        totals[arm]["native_play_seconds"] = round(totals[arm]["native_play_seconds"], 2)
    return dict(pairs=pairs, totals=totals)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seeds", type=int, nargs="+", default=SEEDS)
    args = parser.parse_args()
    result = summarize(args.runs, args.seeds)
    save(args.output, result)
    print(json.dumps(result["totals"], indent=2))
