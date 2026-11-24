"""Compare two sets of native 5v5 matches that share the same kickoff states."""

import argparse
from collections import Counter
from itertools import groupby
import json
from pathlib import Path

import numpy as np

from audit_five_roles import audit, load_match
from roboturk.timed_motion import INTERPOLATION_MARGIN


def profiled_motion(folder, game, states):
    """Count braking and sustained stalls from per-robot native match telemetry."""
    dt = game["dt"]
    frames = game["frames"]
    if states is None:
        _, states = load_match(folder)
    xy = states[:, 4:].reshape(len(frames), len(game["keys"]), 3)[..., :2]
    config = json.loads((folder / "config.json").read_text())
    clearance = config["open_play_policy"]["clearance"] + INTERPOLATION_MARGIN
    brakes_by_robot, brakes_by_role, brakes_by_control = Counter(), Counter(), Counter()
    planned_by_control, planned_by_role = Counter(), Counter()
    inside_buffer = 0
    stalls, shortcuts = [], []
    counts = Counter()
    for index, (team, number) in enumerate(game["keys"]):
        jersey = f"{team} {number}"
        labels = []
        for tick, frame in enumerate(frames):
            move = frame["motion"][index]
            role = frame["roles"][index]
            stalled = (
                move
                and move["speed_mps"] < 0.08
                and np.linalg.norm(xy[tick, index] - move["target"]) > 0.15
            )
            labels.append((role, frame["restart"]) if stalled else None)
            if move is None:
                continue
            counts[move["mode"]] += 1
            control = frame["control"]
            if control is None:
                possession = "loose"
            else:
                possession = "own team" if game["keys"][control][0] == team else "opponent"
            planned_by_control[possession] += 1
            planned_by_role[role] += 1
            if move["mode"] == "shortcut":
                shortcuts.append(dict(tick=tick, robot=jersey, role=role))
            if move["mode"] == "brake":
                brakes_by_robot[jersey] += 1
                brakes_by_role[role] += 1
                brakes_by_control[possession] += 1
                others = xy[tick, np.arange(len(game["keys"])) != index]
                inside_buffer += int(
                    np.any(np.linalg.norm(others - xy[tick, index], axis=1) < clearance)
                )
        for label, group in groupby(enumerate(labels), key=lambda item: item[1]):
            if label is None:
                continue
            indices = [i for i, _ in group]
            if len(indices) >= round(0.60 / dt):
                stalls.append(
                    dict(robot=jersey, role=label[0], start=indices[0], ticks=len(indices))
                )
    if counts != Counter(game["planner_decisions"]):
        raise ValueError(f"Per-player planner counts do not match the summary in {folder}")
    return dict(
        confirmed_control_s=round(sum(f["control"] is not None for f in frames) * dt, 3),
        brakes_by_robot=dict(brakes_by_robot),
        brakes_by_role=dict(brakes_by_role),
        brakes_by_control=dict(brakes_by_control),
        planned_by_control=dict(planned_by_control),
        planned_by_role=dict(planned_by_role),
        brakes_inside_buffer=inside_buffer,
        stalls=stalls,
        stalled_robot_seconds=round(sum(row["ticks"] for row in stalls) * dt, 3),
        shortcuts=shortcuts,
    )


def summarize(folder, game, states=None):
    report = audit(folder)
    clearance = game["robot_clearance"]
    summary = dict(
        score=game["score"],
        passes=report["passes"],
        completed_passes=len(report["confirmed_passes"]),
        receptions=report["confirmed_passes"],
        planner_decisions=game["planner_decisions"],
        contact_episodes=clearance["contact_episodes"],
        min_robot_gap_m=clearance["min_gap_m"],
    )
    if game["frames"] and "motion" in game["frames"][0]:
        summary["motion"] = profiled_motion(folder, game, states)
    return summary


def compare(released, updated):
    pairs = []
    for folder in sorted(released.iterdir()):
        counterpart = updated / folder.name
        if not folder.is_dir():
            continue
        old, before = load_match(folder)
        new, after = load_match(counterpart)
        for key in ("seed", "seconds", "dt", "keys", "sign"):
            if old[key] != new[key]:
                raise ValueError(f"Different {key} for kickoff {folder.name}")
        if not np.array_equal(before[0], after[0]) or (
            old["frames"][0]["opening_velocities"] != new["frames"][0]["opening_velocities"]
        ):
            raise ValueError(f"Different native opening state for kickoff {folder.name}")
        pairs.append(
            dict(
                seed=old["seed"],
                seconds=old["seconds"],
                released=summarize(folder, old, before),
                updated=summarize(counterpart, new, after),
            )
        )
    if not pairs or len({pair["seed"] for pair in pairs}) != len(pairs):
        raise ValueError("Use at least one matched kickoff with distinct seeds")
    if len(pairs) != sum(child.is_dir() for child in updated.iterdir()):
        raise ValueError("Both sets must contain the same kickoffs")
    totals = {}
    for arm in ("released", "updated"):
        rows = [pair[arm] for pair in pairs]
        decisions = Counter()
        for row in rows:
            decisions.update(row["planner_decisions"])
        totals[arm] = dict(
            goals=sum(sum(row["score"]) for row in rows),
            passes=sum(row["passes"] for row in rows),
            completed_passes=sum(row["completed_passes"] for row in rows),
            contact_episodes=sum(row["contact_episodes"] for row in rows),
            matches_with_contact=sum(row["contact_episodes"] > 0 for row in rows),
            min_robot_gap_m=min(row["min_robot_gap_m"] for row in rows),
            planner_decisions=dict(decisions),
        )
        if all("motion" in row for row in rows):
            totals[arm].update(
                sustained_stalls=sum(len(row["motion"]["stalls"]) for row in rows),
                stalled_robot_seconds=round(
                    sum(row["motion"]["stalled_robot_seconds"] for row in rows), 3
                ),
                confirmed_control_s=round(
                    sum(row["motion"]["confirmed_control_s"] for row in rows), 3
                ),
            )
    return dict(pairs=pairs, totals=totals)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("released", type=Path)
    parser.add_argument("updated", type=Path)
    args = parser.parse_args()
    print(json.dumps(compare(args.released, args.updated), indent=2))
