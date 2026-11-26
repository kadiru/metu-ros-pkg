#!/usr/bin/env python3
"""Compare native matched-start panels and verify physical chain receipts."""

import argparse
import json
from pathlib import Path
import numpy as np
from roboturk.contest_metrics import measures

ROOT = Path(__file__).resolve().parents[1]
RELEASE = ROOT / "policies/contest_outlet_v1"


def read(path):
    return json.loads(path.read_text())


def panel(folder, plan):
    cfg = read(folder / "launch.json")
    demo = cfg["mode"] == "demo"
    first = plan["demo_first" if demo else "reserved_first"]
    count = plan["demo_matches" if demo else "matches_per_panel"]
    assert (cfg["first"], cfg["matches"], cfg["seconds"]) == (first, count, plan["seconds"])
    summaries = []
    for i in range(count):
        d = read(folder / f"game-{i:03}.json")
        assert d["seed"] == first + i and d["seconds"] == plan["seconds"]
        assert all(e["event"] != "shot clock" for e in d["events"])
        states = np.load(folder / f"trace-{i:03}.npz")["states"]
        assert states.shape == (plan["seconds"] * 100, 22)
        m = measures(states, d)
        assert m == d["contest_metrics"]
        # Independent mouth, separation and bounds checks on all six robots.
        robots = states[:, 4:].reshape(-1, 6, 3)
        contacts = np.zeros(len(states), bool)
        for a in range(6):
            for b in range(a):
                contacts |= np.linalg.norm(robots[:, a, :2] - robots[:, b, :2], axis=1) < 0.18
        exits = ((np.abs(robots[:, :, 0]) > 4.41) | (np.abs(robots[:, :, 1]) > 2.91)).any(1)
        assert contacts.sum() == d["stats"]["contacts"] and exits.sum() == d["stats"]["exits"]
        own_chains = 0
        supported = 0
        for e in m["contests"]:
            if "received" not in e:
                continue
            team = "blue" if e["winner"] < 3 else "yellow"
            own_chains += team == d["challenger"]
            options = [
                a
                for a in d["learned_actions"]
                if a["head"] == "outlet"
                and a["team"] == team
                and a["actor"] == e["winner"] % 3
                and e["start"] <= 4 * a["tick"] <= e["recovery"]
            ]
            supported += bool(options and options[-1]["mate"] == e["receiver"] % 3)
            # Verify the kick, recipient identity and confirmed sole-hold samples
            # independently of the event extraction state machine.
            actor = e["winner"]
            receiver = e["receiver"]
            kick_tick = e["pass_tick"]
            kicks = [k for k in d["events"] if k["event"] == "kick" and k["tick"] == kick_tick]
            assert len(kicks) == 1 and kicks[0]["team"] == team and kicks[0]["robot"] == actor % 3
            assert kicks[0]["speed"] == 2.2 and 0 <= kick_tick * 4 - e["recovery"] <= 800
            delta = states[:, None, :2] - robots[:, :, :2]
            yaw = robots[:, :, 2]
            x = delta[:, :, 0] * np.cos(yaw) + delta[:, :, 1] * np.sin(yaw)
            y = -delta[:, :, 0] * np.sin(yaw) + delta[:, :, 1] * np.cos(yaw)
            held = (np.abs(x - 0.115) < 0.035) & (np.abs(y) < 0.045)
            start = e["recovery"] - 19
            assert (held[start : start + 20, actor] & (held[start : start + 20].sum(1) == 1)).all()
            end = (e["received"] + 1) * 4
            mask = held[kick_tick * 4 : end, receiver] & (held[kick_tick * 4 : end].sum(1) == 1)
            assert (np.convolve(mask.astype(int), np.ones(20, int), "valid") == 20).any()
        summaries.append(
            dict(
                seed=d["seed"],
                contested_seconds=m["contested_seconds"],
                contests=len(m["contests"]),
                recoveries=m["recoveries"],
                chains=m["chains"],
                designated_outlet_chains=supported,
                challenger_chains=own_chains,
                late=m["late"],
                completions=m["completions"],
                passes=m["passes"],
                contacts=int(contacts.sum()),
                exits=int(exits.sum()),
                samples=len(states),
                goals=sum(d["score"].values()),
            )
        )
    totals = {k: sum(g[k] for g in summaries) for k in summaries[0] if k != "seed"}
    totals["late_matches"] = sum(g["late"] > 0 for g in summaries)
    totals["chain_matches"] = sum(g["chains"] > 0 for g in summaries)
    return dict(totals=totals, games=summaries)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("folder", type=Path)
    p.add_argument("--demo", action="store_true")
    a = p.parse_args()
    protocol = read(RELEASE / "protocol.json")
    if a.demo:
        output = panel(a.folder, protocol)
        output["passed"] = output["totals"]["chains"] > 0
    else:
        panels = {k: panel(a.folder / k, protocol) for k in protocol["panels"]}
        # Geometry of the observed first frame must match across treatments.
        for i in range(protocol["matches_per_panel"]):
            starts = [np.load(a.folder / k / f"trace-{i:03}.npz")["states"][0] for k in panels]
            for state in starts[1:]:
                np.testing.assert_allclose(state, starts[0], atol=1e-7, rtol=0)
        t, b, h = [panels[k]["totals"] for k in ("trained", "released", "head_to_head")]
        g = protocol["gates"]
        checks = dict(
            contested_time=t["contested_seconds"] < b["contested_seconds"]
            and t["contested_seconds"]
            <= b["contested_seconds"] * g["max_trained_contested_seconds_relative_to_released"],
            chains=t["chains"] >= g["min_trained_chains"],
            chain_matches=t["chain_matches"] >= g["min_trained_matches_with_chain"],
            late=t["late_matches"] >= g["min_trained_matches_with_late_reception"],
            head_to_head=h["challenger_chains"] >= g["min_trained_team_chains_head_to_head"],
            safety=all(
                v["totals"]["exits"] <= g["max_robot_field_exits"]
                and v["totals"]["contacts"] / v["totals"]["samples"] <= g["max_contact_fraction"]
                for v in panels.values()
            ),
        )
        output = dict(passed=all(checks.values()), checks=checks, panels=panels)
    (a.folder / "audit.json").write_text(json.dumps(output, indent=2) + "\n")
    print(output.get("checks", output.get("totals")), flush=True)
    return int(not output["passed"])


if __name__ == "__main__":
    raise SystemExit(main())
