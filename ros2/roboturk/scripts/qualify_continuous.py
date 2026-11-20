#!/usr/bin/env python3
"""Apply the published continuous-match protocol without selecting or dropping cases."""

import argparse
from datetime import datetime
import json
from pathlib import Path
import runpy
from roboturk.continuous import summary
from roboturk.continuous_training import digest
from roboturk.self_play_training import write

ROOT = Path(__file__).resolve().parents[1]
AUDIT = runpy.run_path(str(ROOT / "scripts/audit_continuous_release.py"))["audit_file"]


def qualify(release):
    frozen = json.loads((release / "freeze.json").read_text())
    publication = json.loads((release / "freeze-publication.json").read_text())
    assert publication["freeze_sha256"] == digest(release / "freeze.json")
    assert publication["remote_hashes_verified"] and publication["commit"]
    for path, expected in frozen["runtime_sha256"].items():
        assert digest(ROOT / path) == expected, path
    for path, expected in frozen["artifact_sha256"].items():
        assert digest(release / path) == expected, path
    protocol = frozen["protocol"]
    assert digest(release / "protocol.json") == frozen["protocol_sha256"]
    models = {}
    for selection in frozen["selection"]:
        for label, index in [("initial", 0), ("final", -1)]:
            models[f"{selection['seed']}-{label}"] = selection["snapshots"][index]
    panels, grouped, attacks = {}, {611: [], 622: []}, {611: set(), 622: set()}
    for panel in protocol["panels"]:
        path = release / "qualification" / (panel["name"] + ".json")
        raw = json.loads(path.read_text())
        receipt = json.loads(path.with_suffix(".launch.json").read_text())
        assert datetime.fromisoformat(receipt["started_at"]) > datetime.fromisoformat(
            frozen["created_at"]
        )
        for relative, expected in receipt["runtime_sha256"].items():
            assert expected == frozen["runtime_sha256"][relative]
        for color in ("blue", "yellow"):
            expected = models[panel[color]]
            for field in ("sha256", "metadata_sha256", "actor_sha256", "contract"):
                assert raw["models"][color][field] == expected[field], (panel["name"], color, field)
        assert raw["duration"] == protocol["duration"]
        assert "unfinished" in raw["summary"]
        audited = AUDIT(path, panel["seed"], panel["count"])
        panels[panel["name"]] = audited
        grouped[panel["training_seed"]].extend(raw["matches"])
        attacks[panel["training_seed"]].update(e["seed"] for e in audited["counterattacks"])
    results = {}
    gates = protocol["gates_per_training_seed_60_games"]
    for seed, matches in grouped.items():
        assert len(matches) == 60 and len({m["seed"] for m in matches}) == 60
        s = summary(matches)
        loose = [m for m in matches if m["start"] == "loose"]
        metrics = {
            "completion_fraction": s["completed"] / len(matches),
            "mean_contact_fraction": s["mean_contact_fraction"],
            "field_exit_fraction": s["field_exit_matches"] / len(matches),
            "loose_start_acquisition_fraction": sum(
                sum(m["acquisitions"].values()) > 0 for m in loose
            )
            / len(loose),
            "goals": sum(s["goals"].values()),
            "goals_per_color": min(s["goals"].values()),
            "passes": s["passes"],
            "receptions": s["receptions"],
            "shots": sum(sum(m["shots"].values()) for m in matches),
            "counterattack_matches": len(attacks[seed]),
            "mean_support_distance_per_color_m": min(
                sum(m["support_distance"].get(t, 0) for m in matches) / len(matches)
                for t in ("blue", "yellow")
            ),
            "all_event_and_hash_audits_pass": True,
        }
        decisions = {}
        for key, limit in gates.items():
            if key.startswith("min_"):
                value = metrics[key[4:]]
                passed = value >= limit
            elif key.startswith("max_"):
                value = metrics[key[4:]]
                passed = value <= limit
            else:
                value = metrics[key]
                passed = value == limit
            decisions[key] = {"value": value, "limit": limit, "passed": bool(passed)}
        results[str(seed)] = {
            "passed": all(d["passed"] for d in decisions.values()),
            "summary": s,
            "metrics": metrics,
            "gates": decisions,
        }
    return {
        "passed": all(r["passed"] for r in results.values()),
        "freeze_sha256": digest(release / "freeze.json"),
        "frozen_commit": publication["commit"],
        "panels": panels,
        "training_seeds": results,
        "cases": sum(len(v) for v in grouped.values()),
        "primary_seed": protocol["primary_seed"],
        "reserved_cases_selected_or_dropped": 0,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--release", type=Path, required=True)
    args = p.parse_args()
    result = qualify(args.release.resolve())
    write(args.release / "qualification-audit.json", result)
    print(json.dumps(result["training_seeds"], indent=2))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
