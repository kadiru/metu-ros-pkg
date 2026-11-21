"""Replay the actual decisions and test the same observations with the central lane cleared."""

import argparse
from collections import Counter
from dataclasses import replace
from pathlib import Path
import numpy as np
from roboturk.blocked_audit import opening_result
from roboturk.blocked_outlet import geometry, select
from roboturk.model import Ball, Robot, RobotKey, World
from roboturk.three_match import KEYS
from run_recovery_attack import read, save


def replay(folder, checkpoint):
    cfg = read(folder / "launch.json")
    p = read(checkpoint)
    counts = Counter()
    slots = Counter()
    scorers = Counter()
    rows = []
    for i, (seed, _, _) in enumerate(cfg["schedule"]):
        g = read(folder / f"game-{i:03}.json")
        states = np.load(folder / f"trace-{i:03}.npz")["states"]
        assert g["seed"] == seed and len(states) == 100 * cfg["seconds"]
        row = opening_result(states, g, cfg["protocol"])
        assert row == g["opening"]
        counts[row["outcome"]] += 1
        if "decision" not in row:
            continue
        ch = row["chain"]
        team = ch["team"]
        frame = states[4 * row["decision"] - 1]
        robots = tuple(Robot(k, *r) for k, r in zip(KEYS, frame[4:].reshape(6, 3), strict=True))
        w = World(0, robots, Ball(*frame[:4]), attack_sign=1 if seed % 2 == 0 else -1)
        actor = w.robot(RobotKey(team, row["passer"]))
        mate = w.robot(RobotKey(team, row["receiver"]))
        choice, action, _ = select(w, actor, p)
        assert action == "pass" and choice.receiver == mate.key.id
        if row["qualified"]:
            slots[choice.slot] += 1
            scorers[team] += 1
        direction = w.direction(team)
        other = next(r for r in robots if r.key.team == team and r.key not in (mate.key, actor.key))
        rivals = [(r.x, r.y) for r in robots if r.key.team != team]
        _, other_lane, other_space = geometry(
            (w.ball.x, w.ball.y), direction, (other.x, other.y), rivals
        )
        other_open = (
            other_lane >= cfg["protocol"]["minimum_pass_lane_gap_m"]
            and other_space >= cfg["protocol"]["minimum_receiver_space_m"]
        )
        moved = []
        comparison = []
        for r in robots:
            if (
                r.key.team != team
                and geometry((w.ball.x, w.ball.y), direction, (mate.x, mate.y), [(r.x, r.y)])[0]
                <= cfg["protocol"]["blocked_center_gap_m"]
            ):
                r = replace(r, x=w.ball.x - 2.8 * direction, y=2.62 if r.y >= w.ball.y else -2.62)
                moved.append(r.key.id)
            comparison.append(r)
        _, alternative, _ = select(replace(w, robots=tuple(comparison)), actor, p)
        rows.append(
            dict(
                seed=seed,
                qualified=row["qualified"],
                receiver=mate.key.id,
                slot=choice.slot,
                other_receiver=other.key.id,
                other_open=other_open,
                moved_defenders=moved,
                observed=action,
                cleared_lane=alternative,
            )
        )
    measured = [r for r in rows if r["moved_defenders"]]
    return dict(
        games=len(cfg["schedule"]),
        outcomes=dict(counts),
        qualifying_slots=dict(slots),
        qualifying_sides=dict(scorers),
        unique_open_receiver_goals=sum(r["qualified"] and not r["other_open"] for r in rows),
        replayed_passes=len(rows),
        cleared_frames=len(measured),
        switched_from_pass=sum(r["cleared_lane"] != "pass" for r in measured),
        decisions=rows,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder", type=Path)
    p.add_argument("checkpoint", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    save(a.output, replay(a.folder, a.checkpoint))
