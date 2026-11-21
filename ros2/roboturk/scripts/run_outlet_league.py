#!/usr/bin/env python3
"""Evaluate the return-trained team in native development, paired matches or a demo."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
from audit_contest import panel
from run_contest import ROOT

V1 = ROOT / "policies/contest_outlet_v1"
V2 = ROOT / "policies/contest_outlet_v2"
WORLDS = ROOT / "src/roboturk_ros/webots/worlds"


def read(p):
    return json.loads(p.read_text())


def save(p, data):
    p.write_text(json.dumps(data, indent=2) + "\n")


def verify(freeze):
    for path, digest in read(freeze).items():
        assert hashlib.sha256((ROOT / path).read_bytes()).hexdigest() == digest, path


def audit(folder, plan, demo):
    if demo:
        summary = panel(folder, plan)
        summary["passed"] = summary["totals"]["chains"] > 0
    else:
        panels = {k: panel(folder / k, plan) for k in plan["panels"]}
        for i in range(plan["matches_per_panel"]):
            starts = [np.load(folder / k / f"trace-{i:03}.npz")["states"][0] for k in panels]
            for other in starts[1:]:
                np.testing.assert_allclose(starts[0], other, atol=1e-7, rtol=0)
        b, t, h = (panels[k]["totals"] for k in plan["panels"])
        g = plan["gates"]
        checks = dict(
            contested_time=t["contested_seconds"] < b["contested_seconds"]
            and t["contested_seconds"]
            <= b["contested_seconds"] * g["max_trained_contested_seconds_relative_to_released"],
            chains=t["chains"] >= g["min_trained_chains"],
            chain_matches=t["chain_matches"] >= g["min_trained_matches_with_chain"],
            late=t["late_matches"] >= g["min_trained_matches_with_late_reception"],
            head_to_head=h["challenger_chains"] >= g["min_trained_team_chains_head_to_head"],
            safety=all(
                p["totals"]["exits"] <= g["max_robot_field_exits"]
                and p["totals"]["contacts"] / p["totals"]["samples"] <= g["max_contact_fraction"]
                for p in panels.values()
            ),
        )
        summary = dict(passed=all(checks.values()), checks=checks, panels=panels)
    save(folder / "audit.json", summary)
    print(summary.get("checks", summary.get("totals")), flush=True)
    return summary["passed"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "mode", choices=["train_jaw", "develop", "qualify", "demo", "audit", "audit-demo"]
    )
    parser.add_argument("folder", type=Path)
    parser.add_argument(
        "--webots",
        type=Path,
        default=ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots",
    )
    parser.add_argument("--model", type=Path, default=V2 / "policy.json")
    parser.add_argument("--development-offset", type=int, default=0)
    a = parser.parse_args()
    plan = read(V2 / "protocol.json")
    folder = a.folder.resolve()
    if a.mode.startswith("audit"):
        return int(not audit(folder, plan, a.mode == "audit-demo"))
    verify(V1 / "freeze.json")
    verify(V1 / "isolation-freeze.json")
    if a.mode not in ("develop", "train_jaw"):
        verify(V2 / "freeze.json")
        assert a.model.resolve() == V2 / "policy.json" and a.development_offset == 0
    assert 0 <= a.development_offset < 20000
    folder.mkdir(parents=True, exist_ok=False)
    binary = a.webots.resolve()
    meta = dict(
        root=str(ROOT),
        webots_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
        model=str(a.model.resolve()),
        seed=917,
    )

    def simulate(out, mode, index=None):
        env = dict(
            os.environ,
            CONTEST_RUN=str(out),
            PYTHONPATH=str(ROOT / "src/roboturk_core"),
            PATH=str(Path(sys.executable).parent) + os.pathsep + os.environ["PATH"],
        )
        cmd = [str(binary), "--batch", "--stdout", "--stderr", "--mode=fast"]
        if mode != "demo":
            cmd += ["--no-rendering", "--minimize"]
        if index is not None:
            env["CONTEST_INDEX"] = str(index)
        cmd.append(str(WORLDS / "ssl_outlet_league.wbt"))
        with (out / "webots.log").open("a") as log:
            subprocess.run(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)

    if a.mode == "train_jaw":
        save(
            folder / "launch.json",
            dict(**meta, mode="train_jaw", first=8190000, matches=256, seconds=8),
        )
        simulate(folder, "train_jaw")
        assert len(read(folder / "results.json")) == 256
        return 0
    if a.mode == "develop":
        first = plan["development_first"] + a.development_offset
        for name, mode, offset, count, seconds, duel in [
            ("develop", "trained", 0, 12, 10, True),
            ("duel-mixed", "head_to_head", 200, 16, 10, True),
            ("head_to_head", "head_to_head", 100, 8, 45, False),
            ("ordinary", "trained", 300, 8, 60, False),
        ]:
            out = folder / name
            out.mkdir()
            save(
                out / "launch.json",
                dict(
                    **meta,
                    mode=mode,
                    first=first + offset,
                    matches=count,
                    seconds=seconds,
                    duel=duel,
                ),
            )
            simulate(out, mode)
            assert len(read(out / "results.json")) == count
        return 0
    if a.mode == "demo":
        save(
            folder / "launch.json",
            dict(
                **meta,
                mode="demo",
                first=plan["demo_first"],
                matches=plan["demo_matches"],
                seconds=plan["seconds"],
            ),
        )
        simulate(folder, "demo")
        return int(not audit(folder, plan, True))
    count = plan["matches_per_panel"]
    for mode in plan["panels"]:
        out = folder / mode
        out.mkdir()
        save(
            out / "launch.json",
            dict(
                **meta,
                mode=mode,
                first=plan["reserved_first"],
                matches=count,
                seconds=plan["seconds"],
                process_per_match=True,
            ),
        )
    for i in range(count):
        for mode in plan["panels"]:
            out = folder / mode
            simulate(out, mode, i)
            result = read(out / f"game-{i:03}.json")
            print(i, mode, result["contest_metrics"]["chains"], flush=True)
    for mode in plan["panels"]:
        out = folder / mode
        save(
            out / "results.json",
            [read(out / f"game-{i:03}.json")["contest_metrics"] for i in range(count)],
        )
    return int(not audit(folder, plan, False))


if __name__ == "__main__":
    raise SystemExit(main())
