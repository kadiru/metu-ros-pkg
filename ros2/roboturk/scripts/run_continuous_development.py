#!/usr/bin/env python3
"""Fresh development panels against self and own history; no cross-seed training."""

import argparse
import json
from pathlib import Path
import runpy
import subprocess
import sys
from roboturk.continuous import summary
from roboturk.continuous_training import descriptor, load
from roboturk.self_play_training import write

ROOT = Path(__file__).resolve().parents[1]
audit_file = runpy.run_path(str(ROOT / "scripts/audit_continuous_release.py"))["audit_file"]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--webots", required=True)
    p.add_argument("--league611", type=Path, required=True)
    p.add_argument("--league622", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--seed", type=int, default=4401000)
    args = p.parse_args()
    if args.output.exists():
        p.error("Use a new development output directory")
    if not 4400000 <= args.seed < 4499600:
        p.error("Development seeds only")
    protocol = []
    for index, seed in enumerate((611, 622)):
        league = getattr(args, f"league{seed}").resolve()
        ledger = json.loads((league / f"seed{seed}/league.json").read_text())
        assert len(ledger["updates"]) == 2 and len(ledger["snapshots"]) == 3
        final, initial = [Path(ledger["snapshots"][i]["path"]) for i in (-1, 0)]
        assert ledger["updates"][-1]["safety_contract"] == "roboturk.velocity_safety.v1"
        for j, (label, opponent) in enumerate((("self", final), ("history", initial))):
            protocol.append(
                {
                    "name": f"{seed}-{label}",
                    "training_seed": seed,
                    "seed": args.seed + (2 * index + j) * 100,
                    "count": 15,
                    "blue": descriptor(final, load(final)),
                    "yellow": descriptor(opponent, load(opponent)),
                }
            )
    args.output.mkdir(parents=True)
    write(
        args.output / "panels.json",
        {
            "purpose": "development, not reserved qualification",
            "selection": "last completed generation; no other-seed opponent in these panels",
            "panels": protocol,
        },
    )
    grouped = {611: [], 622: []}
    attacks = {611: set(), 622: set()}
    for panel in protocol:
        dest = args.output / panel["name"]
        command = [
            sys.executable,
            str(ROOT / "scripts/run_continuous.py"),
            "--webots",
            args.webots,
            "--mode",
            "evaluate",
            "--checkpoint",
            panel["blue"]["path"],
            "--opponent",
            panel["yellow"]["path"],
            "--seed",
            str(panel["seed"]),
            "--episodes",
            str(panel["count"]),
            "--duration",
            "45",
            "--velocity-safety",
            "--output",
            str(dest),
            "--timeout",
            "1800",
        ]
        print("Development", panel["name"], "starting", flush=True)
        with dest.with_suffix(".log").open("x") as log:
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        raw = json.loads(dest.with_suffix(".json").read_text())
        audit = audit_file(dest.with_suffix(".json"), panel["seed"], panel["count"], True)
        assert all(
            m["neural_inference_batches"] == {"blue": m["steps"], "yellow": m["steps"]}
            for m in raw["matches"]
        )
        write(dest.with_suffix(".audit.json"), audit)
        grouped[panel["training_seed"]].extend(raw["matches"])
        attacks[panel["training_seed"]].update(e["seed"] for e in audit["counterattacks"])
        print("Development", panel["name"], raw["summary"], flush=True)
    report = {}
    for seed, matches in grouped.items():
        s = summary(matches)
        loose = [m for m in matches if m["start"] == "loose"]
        metrics = {
            "completion_fraction": s["completed"] / 30,
            "contact_fraction": s["mean_contact_fraction"],
            "field_exit_fraction": s["field_exit_matches"] / 30,
            "loose_acquisition_fraction": sum(sum(m["acquisitions"].values()) > 0 for m in loose)
            / len(loose),
            "goals": sum(s["goals"].values()),
            "goals_per_color": min(s["goals"].values()),
            "passes": s["passes"],
            "receptions": s["receptions"],
            "shots": sum(sum(m["shots"].values()) for m in matches),
            "counterattack_matches": len(attacks[seed]),
            "mean_support_m_per_color": min(
                sum(m["support_distance"].get(t, 0) for m in matches) / 30
                for t in ("blue", "yellow")
            ),
            "safety_intervention_tick_fraction": sum(
                len(m["safety_interventions"]) for m in matches
            )
            / sum(m["steps"] for m in matches),
        }
        minimum = {
            "completion_fraction": 0.95,
            "loose_acquisition_fraction": 0.9,
            "goals": 10,
            "goals_per_color": 3,
            "passes": 10,
            "receptions": 8,
            "shots": 10,
            "counterattack_matches": 5,
            "mean_support_m_per_color": 1,
        }
        maximum = {"contact_fraction": 0.02, "field_exit_fraction": 0.05}
        passed = all(metrics[k] >= v for k, v in minimum.items()) and all(
            metrics[k] <= v for k, v in maximum.items()
        )
        report[str(seed)] = {
            "summary": s,
            "metrics": metrics,
            "minimum": minimum,
            "maximum": maximum,
            "passed": passed,
        }
    write(args.output / "report.json", report)
    print(json.dumps(report, indent=2), flush=True)
    raise SystemExit(0 if all(r["passed"] for r in report.values()) else 1)


if __name__ == "__main__":
    main()
