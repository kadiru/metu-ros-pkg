#!/usr/bin/env python3
"""Run every predeclared panel once, retain all outcomes, independently audit."""

import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys
from audit_strategy import audit

ROOT = Path(__file__).resolve().parents[1]
RELEASE = ROOT / "policies/strategy_self_play_v1"


def gates(records, audits, protocol):
    g = protocol["gates_per_seed_60_matches"]
    count = len(records)
    outcomes = Counter(e["type"] for m in records for e in m["option_events"])
    scores = {t: sum(m["score"].get(t, 0) for m in records) for t in ("blue", "yellow")}
    ticks = {
        k: sum(
            n for m in records for mode, n in m["strategy_ticks"].items() if mode.endswith("/" + k)
        )
        for k in ("receive", "redirect", "clear")
    }
    completion = sum(m["reason"] == "time" for m in records) / max(1, count)
    contact = sum(m["contact_fraction"] for m in records) / max(1, count)
    exits = sum(m["outside_checks"] > 0 for m in records)
    checks = {
        "complete_panel": count == 60,
        "completion": completion >= g["min_completion_fraction"],
        "contacts": contact <= g["maximum_mean_contact_fraction"],
        "field_exits": exits <= g["maximum_field_exit_matches"],
        "goals": sum(scores.values()) >= g["minimum_total_goals"],
        "both_colors_score": min(scores.values()) >= g["minimum_goals_per_color"],
        "receive_stop": outcomes["receive_stop"] >= g["minimum_receive_stop"],
        "redirect_kick": outcomes["redirect_kick"] >= g["minimum_redirect_kicks"],
        "redirect_goal": outcomes["redirect_goal"] >= g["minimum_redirect_goals"],
        "clearance_complete": outcomes["clearance_complete"] >= g["minimum_clearance_complete"],
        "skill_use": min(ticks.values()) >= g["minimum_each_new_skill_ticks"],
        "independent_audit": all(a["passed"] for a in audits),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "matches": count,
        "completion_fraction": completion,
        "mean_contact_fraction": contact,
        "field_exit_matches": exits,
        "goals": scores,
        "new_skill_ticks": ticks,
        "skill_outcomes": {
            k: outcomes[k]
            for k in ("receive_stop", "redirect_kick", "redirect_goal", "clearance_complete")
        },
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--webots", required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error("Keep all previous qualification attempts")
    a.output.mkdir(parents=True)
    protocol = json.loads((RELEASE / "protocol.json").read_text())
    model_map = {
        f"{s}-{stage}": RELEASE / f"seed{s}" / folder / "policy.zip"
        for s in (811, 822)
        for stage, folder in [("initial", "initial"), ("final", "generation2")]
    }
    records, audits = {}, {}
    for panel in protocol["panels"]:
        output = a.output / panel["name"]
        command = [
            sys.executable,
            str(ROOT / "scripts/run_strategy.py"),
            "--webots",
            a.webots,
            "--mode",
            "evaluate",
            "--checkpoint",
            str(model_map[panel["blue"]]),
            "--opponent",
            str(model_map[panel["yellow"]]),
            "--output",
            str(output),
            "--episodes",
            str(panel["count"]),
            "--seed",
            str(panel["seed"]),
            "--duration",
            str(protocol["native_duration_seconds"]),
            "--freeze",
            str(RELEASE / "freeze.json"),
            "--timeout",
            "3600",
        ]
        with output.with_suffix(".log").open("w") as log:
            subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
        data = json.loads(output.with_suffix(".json").read_text())
        assert [m["seed"] for m in data["matches"]] == list(
            range(panel["seed"], panel["seed"] + panel["count"])
        )
        result = audit(output.with_suffix(".json"))
        output.with_suffix(".audit.json").write_text(json.dumps(result, indent=2) + "\n")
        records[panel["name"]] = data["matches"]
        audits[panel["name"]] = result
        print(panel["name"], data["summary"], "audit", result["passed"], flush=True)
    result = {
        str(s): gates(
            [m for name, matches in records.items() if name.startswith(str(s)) for m in matches],
            [v for name, v in audits.items() if name.startswith(str(s))],
            protocol,
        )
        for s in (811, 822)
    }
    (a.output / "qualification.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result), flush=True)
    return 0 if all(r["passed"] for r in result.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
