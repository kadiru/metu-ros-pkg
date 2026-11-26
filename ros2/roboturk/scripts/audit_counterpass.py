"""Measure real lead passes, paired goals and command-prefix equivalence."""

import argparse
from collections import Counter
from pathlib import Path
import numpy as np
from compare_takeaways import all_games, compare, trace
from train_counterpass import read, save


def pass_detail(g, s, r):
    if not r.get("passes"):
        return None
    target = next(
        (v for v in g["takeaway_actions"] if v["sample"] == r["sample"] and "target" in v), None
    )
    if not target:
        return None
    p = next(v for v in g["passes"] if v["tick"] == r["passes"][0])
    if p["receiver"] != target["recipient"]:
        return None
    release = 4 * p["tick"] - 1
    side = 3 * int(r["team"] == "yellow")
    actor = side + p["receiver"]
    robots = s[:, 4:].reshape(-1, 6, 3)
    start = robots[release, actor, :2]
    vel = (start - robots[max(0, release - 4), actor, :2]) / 0.04
    ball = s[release, :2]
    ray = np.asarray(target["target"]) - ball
    ray /= max(0.001, np.linalg.norm(ray))
    center = np.asarray(target["target"]) + 0.115 * ray
    offset = center - start
    length = np.linalg.norm(offset)
    heading = offset / max(0.001, length)
    # Arrival is the first of the 20 consecutive exclusive samples before
    # the pass monitor marks completion. Recheck the mouth geometry directly.
    sample = s[release + 1 : (p["end"] + 1) * 4]
    q = sample[:, 4:].reshape(-1, 6, 3)
    d = sample[:, None, :2] - q[:, :, :2]
    co, si = np.cos(q[:, :, 2]), np.sin(q[:, :, 2])
    held = (abs(d[:, :, 0] * co + d[:, :, 1] * si - 0.115) < 0.035) & (
        abs(d[:, :, 1] * co - d[:, :, 0] * si) < 0.045
    )
    sole = (
        held[:, actor]
        & (held.sum(axis=1) == 1)
        & (np.linalg.norm(sample[:, :2] - p["ball"], axis=1) >= 0.55)
    )
    k = (
        int(np.flatnonzero(np.convolve(sole.astype(int), np.ones(20), "valid") == 20)[0])
        + release
        + 1
    )
    adv = float((robots[k, actor, :2] - start) @ heading)
    rivals = robots[k, 3 - side : 6 - side, :2]
    gap = float(np.min(np.linalg.norm(rivals - robots[k, actor, :2], axis=1)))
    return dict(
        seed=g["seed"],
        sample=r["sample"],
        team=r["team"],
        passer=p["passer"],
        receiver=p["receiver"],
        tick=p["tick"],
        arrival=k,
        goal=r["goal"],
        lead_m=float(length),
        advance_m=adv,
        running_mps=float(vel @ heading),
        rival_gap_m=gap,
        verified_lead=bool(length >= 0.10 and adv >= 0.08 and vel @ heading > 0.10),
        open_at_arrival=bool(gap >= 0.35),
    )


def evidence(control, new):
    old = {g["seed"]: (g, f) for g, f in all_games(control)}
    summary = compare(control, new)
    by_arm = {}
    clips = []
    prefixes = []
    for g, f in all_games(control) + all_games(new):
        a, b = g["blue_model"], g["yellow_model"]
        arm = "mixed" if a != b else "trained" if a else "previous"
        s = trace(f)
        if arm != "previous":
            ref, rf = old[g["seed"]]
            assert g["initial_world"] == ref["initial_world"]
            active = [d for d in g["takeaway_actions"] if d.get("distribution")]
            prefix = 4 * (active[0]["sample"] // 4 + 1) if active else len(s)
            assert np.array_equal(s[:prefix], trace(rf)[:prefix])
            prefixes.append(dict(seed=g["seed"], blue=a, yellow=b, samples=prefix))
        for r in g["recoveries"]:
            if r["kind"] != "takeaway":
                continue
            side = (
                arm
                if arm != "mixed"
                else ("challenger_v_old" if g[r["team"] + "_model"] else "old_v_challenger")
            )
            counts = by_arm.setdefault(side, Counter())
            counts["clean"] += 1
            counts["goals"] += int("goal" in r)
            counts["second_half_clean"] += int(r["sample"] >= 4500)
            counts["second_half_goals"] += int(r["sample"] >= 4500 and "goal" in r)
            counts["second_half_pass_goals"] += int(r["sample"] >= 4500 and bool(r.get("passes")))
            attempts = [
                p
                for p in g["passes"]
                if p["team"] == r["team"] and r["sample"] <= 4 * p["tick"] <= r["end"]
            ]
            counts["attempts"] += len(attempts)
            counts["completed"] += sum(
                p["result"] == "complete" and 4 * p["end"] <= r["end"] for p in attempts
            )
            counts["pass_goals"] += int(bool(r.get("passes")))
            for d in g["takeaway_actions"]:
                if d["sample"] == r["sample"] and "distribution" in d:
                    counts["action_" + str(d["distribution"])] += 1
            detail = pass_detail(g, s, r)
            if detail:
                counts["moving_lead_goals"] += int(detail["verified_lead"])
                counts["open_lead_goals"] += int(
                    detail["verified_lead"] and detail["open_at_arrival"]
                )
                if arm == "trained":
                    clips.append(detail | dict(file=f.name))
    summary.update(distribution=by_arm, scoring_passes=clips, verified_initial_prefixes=prefixes)
    stats = summary["distribution"]
    wdl = summary["head_to_head_record"]
    summary["promoted"] = bool(
        stats["trained"]["goals"] > stats["previous"]["goals"]
        and wdl.get("won", 0) >= wdl.get("lost", 0)
        and stats["trained"]["moving_lead_goals"] > 0
    )
    return summary


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("control", type=Path)
    p.add_argument("new", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    for folder, required in ((a.control, 40), (a.new, 56)):
        cfg = read(folder / "launch.json")
        games = all_games(folder)
        assert len(cfg["schedule"]) == len(games) == required
        for (g, f), expected in zip(games, cfg["schedule"], strict=True):
            assert [g["seed"], g["blue_model"], g["yellow_model"]] == expected
            assert len(trace(f)) == 9000
    save(a.output, evidence(a.control, a.new))
