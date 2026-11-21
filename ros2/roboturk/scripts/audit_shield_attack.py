"""Audit v2 held-out and recorded games against the same physical conversion contract."""

import argparse
from pathlib import Path
import numpy as np

from audit_recovery_attack import panel
from run_shield_attack import PLAN, read, save


def audit(folder, demo=False):
    plan = read(PLAN)
    if demo:
        cfg = read(folder / "launch.json")
        assert (cfg["first"], cfg["cases"], cfg["seconds"]) == (
            plan["demo_first"],
            plan["demo_cases"],
            plan["demo_seconds"],
        )
        evidence = panel(folder)
        return dict(**evidence, passed=evidence["totals"]["chains"] > 0)
    panels = {}
    for k, n in [
        ("released", "cases"),
        ("trained", "cases"),
        ("mixed", "mixed_cases"),
        ("ordinary", "ordinary_cases"),
    ]:
        d = folder / k
        cfg = read(d / "launch.json")
        ordinary = k == "ordinary"
        assert (cfg["mode"], cfg["cases"], cfg["first"], cfg["seconds"]) == (
            k,
            plan[n],
            plan["ordinary_first" if ordinary else "first"],
            plan["ordinary_seconds" if ordinary else "seconds"],
        )
        panels[k] = panel(d)
    for i in range(plan["cases"]):
        name = f"trace-{i:03}.npz"
        with np.load(folder / "released" / name) as base:
            for k in ["trained", "mixed"] if i < plan["mixed_cases"] else ["trained"]:
                with np.load(folder / k / name) as v:
                    np.testing.assert_allclose(base["states"][0], v["states"][0], atol=1e-7, rtol=0)
    g = plan["gates"]
    b, t, h, o = (panels[k]["totals"] for k in ("released", "trained", "mixed", "ordinary"))
    checks = dict(
        first=t["first"] >= g["first"],
        improvement=t["first"] - b["first"] >= g["improvement"],
        mixed=h["mixed_first"] >= g["mixed"],
        ordinary=o["chains"] >= g["ordinary"],
        pass_assisted=t["assisted"] >= g["assisted"],
        dribble_assisted=t["carried"] >= g["carried"],
        both_colors=min(t["blue"], t["yellow"]) > 0,
        safety=all(
            v["totals"]["exits"] <= g["max_exits"]
            and v["totals"]["contacts"] / v["totals"]["samples"] <= g["max_contact_fraction"]
            for v in panels.values()
        ),
    )
    return dict(passed=all(checks.values()), checks=checks, panels=panels)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("folder", type=Path)
    p.add_argument("--demo", action="store_true")
    args = p.parse_args()
    result = audit(args.folder, args.demo)
    save(args.folder / "audit.json", result)
    print(result.get("checks", result.get("totals")))
    raise SystemExit(int(not result["passed"]))
