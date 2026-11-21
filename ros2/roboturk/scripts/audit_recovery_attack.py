#!/usr/bin/env python3
"""Audit native ball paths, scoring direction, physical actions and matched starts."""

import argparse
from collections import Counter
from pathlib import Path
import numpy as np
from roboturk.recovery_conversion import conversions
from run_recovery_attack import R, read, save


def panel(folder):
    cfg = read(folder / "launch.json")
    games = []
    for i in range(cfg["cases"]):
        g = read(folder / f"game-{i:03}.json")
        assert g["seed"] == cfg["first"] + i and g["seconds"] == cfg["seconds"]
        assert all(e["event"] != "shot clock" for e in g["events"])
        with np.load(folder / f"trace-{i:03}.npz") as t:
            s = t["states"]
        assert s.shape == (cfg["seconds"] * 100, 22)
        rows = conversions(s, g)
        assert rows == g["conversions"]
        # Reconstruct every referee goal from the endpoint, independently of
        # the possession sequence. The restart event names the next kickoff.
        score = Counter()
        direction = 1 if g["seed"] % 2 == 0 else -1
        for e in g["events"]:
            if e["event"] == "goal":
                b = s[e["tick"] * 4 + 3, :2]
                assert abs(b[0]) > 4.5215 and abs(b[1]) < 0.4785
                score["blue" if b[0] * direction > 0 else "yellow"] += 1
        assert dict(score) == g["score"]
        robots = s[:, 4:].reshape(-1, 6, 3)
        contact = np.zeros(len(s), bool)
        for a in range(6):
            for b in range(a):
                contact |= np.linalg.norm(robots[:, a, :2] - robots[:, b, :2], axis=1) < 0.18
        exits = (np.abs(robots[:, :, 0]) > 4.41) | (np.abs(robots[:, :, 1]) > 2.91)
        assert (
            int(contact.sum()) == g["stats"]["contacts"]
            and int(exits.any(1).sum()) == g["stats"]["exits"]
        )
        won = [e for e in rows if "goal" in e]
        assert len({e["goal"] for e in won}) == len(won)
        for e in won:
            k = e["shot"]
            actor = (0 if e["team"] == "blue" else 3) + e["scorer"]
            # Check a measured velocity change and a ball near the shooter's
            # actual front mouth, independently of the controller's kick event.
            segment = s[max(0, k * 4 - 1) : (k + 1) * 4]
            pose = segment[:, 4 + 3 * actor : 7 + 3 * actor]
            delta = segment[:, :2] - pose[:, :2]
            front = pose[:, :2] + 0.115 * np.stack((np.cos(pose[:, 2]), np.sin(pose[:, 2])), axis=1)
            assert np.linalg.norm(segment[:, :2] - front, axis=1).min() < 0.075
            assert np.linalg.norm(segment[:, 2:4], axis=1).max() > 2.5
            assert np.linalg.norm(delta, axis=1).min() < 0.17
        first = int(bool(rows and "goal" in rows[0]))
        games.append(
            dict(
                seed=g["seed"],
                first=first,
                recoveries=sum("winner" in e for e in rows),
                chains=len(won),
                mixed_first=int(bool(first and rows[0]["team"] == g["challenger"])),
                assisted=sum(bool(e["passes"]) for e in won),
                carried=sum(e["dribble_m"] >= 0.18 for e in won),
                blue=sum(e["team"] == "blue" for e in won),
                yellow=sum(e["team"] == "yellow" for e in won),
                goals=sum(score.values()),
                contacts=int(contact.sum()),
                exits=int(exits.any(1).sum()),
                samples=len(s),
                choices=dict(Counter(o["action"] for o in g["options"])),
            )
        )
    totals = {k: sum(x[k] for x in games) for k in games[0] if k not in ("seed", "choices")}
    totals["choices"] = dict(sum((Counter(x["choices"]) for x in games), Counter()))
    return dict(totals=totals, games=games)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("folder", type=Path)
    p.add_argument("--demo", action="store_true")
    a = p.parse_args()
    plan = read(R / "protocol.json")
    if a.demo:
        cfg = read(a.folder / "launch.json")
        assert (cfg["first"], cfg["cases"], cfg["seconds"]) == (
            plan["demo_first"],
            plan["demo_cases"],
            plan["demo_seconds"],
        )
        result = panel(a.folder)
        result["passed"] = result["totals"]["chains"] > 0
    else:
        panels = {}
        for mode in ("released", "trained", "mixed", "ordinary"):
            d = a.folder / mode
            cfg = read(d / "launch.json")
            n = plan[{"mixed": "mixed_cases", "ordinary": "ordinary_cases"}.get(mode, "cases")]
            assert cfg["mode"] == mode and cfg["cases"] == n
            assert (
                cfg["first"] == plan["ordinary_first" if mode == "ordinary" else "reserved_first"]
            )
            assert cfg["seconds"] == plan["ordinary_seconds" if mode == "ordinary" else "seconds"]
            panels[mode] = panel(d)
        for i in range(plan["cases"]):
            f = f"trace-{i:03}.npz"
            initial = np.load(a.folder / "released" / f)["states"][0]
            for mode in ["trained", "mixed"] if i < plan["mixed_cases"] else ["trained"]:
                np.testing.assert_allclose(
                    initial, np.load(a.folder / mode / f)["states"][0], atol=1e-7, rtol=0
                )
        b, t, h, o = [p["totals"] for p in panels.values()]
        g = plan["gates"]
        checks = dict(
            first=t["first"] >= g["first_conversions"],
            improvement=t["first"] - b["first"] >= g["improvement_over_released"],
            mixed=h["mixed_first"] >= g["mixed_trained_conversions"],
            ordinary=o["chains"] >= g["ordinary_recovery_goals"],
            pass_assisted=t["assisted"] >= g["min_pass_assisted"],
            dribble_assisted=t["carried"] >= g["min_dribble_assisted"],
            both_colors=min(t["blue"], t["yellow"]) > 0,
            safety=all(
                v["totals"]["exits"] <= g["max_exits"]
                and v["totals"]["contacts"] / v["totals"]["samples"] <= g["max_contact_fraction"]
                for v in panels.values()
            ),
        )
        result = dict(passed=all(checks.values()), checks=checks, panels=panels)
    save(a.folder / "audit.json", result)
    print(result.get("checks", result.get("totals")))
    return int(not result["passed"])


if __name__ == "__main__":
    raise SystemExit(main())
