#!/usr/bin/env python3
"""Validate complete native panels, policy identity, coordinator use and chronology."""

import argparse
from datetime import datetime
import gzip
import json
from pathlib import Path
from audit_tdp import audit_file
from stage_tdp import ROOT, RELEASE, KINDS, sha, write


def read(path):
    raw = path.read_bytes()
    return json.loads(gzip.decompress(raw) if path.suffix == ".gz" else raw)


def panel(source, mode):
    protocol = read(RELEASE / "protocol.json")
    launch = read(source.with_suffix(".launch.json"))
    freeze = read(RELEASE / "freeze.json") if mode != "orchestration_development" else None
    if freeze:
        publication = read(RELEASE / "freeze-publication.json")
        assert publication["remote_hashes_verified"]
        assert publication["freeze_sha256"] == sha(RELEASE / "freeze.json")
        assert datetime.fromisoformat(publication["verified_at"]) < datetime.fromisoformat(
            launch["started_at"]
        )
        for path, expected in freeze["files"].items():
            assert sha(ROOT / path) == expected, path
        for path, actual in launch["runtime_sha256"].items():
            assert freeze["files"][path] == actual, path
        assert launch["arguments"]["freeze"]
        assert launch["webots_executable_sha256"] == freeze["webots"]["executable_sha256"]
    reports = {}
    for kind in KINDS:
        seed, last = protocol["tasks"][kind][mode]
        count = last - seed + 1
        path = source / (kind + ".json.gz")
        if not path.exists():
            path = path.with_suffix("")
        data = read(path)
        assert data["control"] == "neural" and data["orchestration"] == "scripted_tdp_playbook.v1"
        checkpoint = data["checkpoint"]
        metadata = read(RELEASE / kind / "policy.json")
        assert checkpoint["sha256"] == sha(RELEASE / kind / "policy.zip")
        assert checkpoint["metadata_sha256"] == sha(RELEASE / kind / "policy.json")
        assert checkpoint["actor_sha256"] == metadata["actor_sha256"]
        assert launch["arguments"]["episodes"] == count
        for e in data["episodes"]:
            assert not e["orchestration_error"]
            assert e["neural_calls"] == e["decision_calls"] == len(e["samples"]) // 4
            events = e["orchestration_events"]
            assert len(events) == 2 and events[0]["event"] == "commit"
            assert events[0]["kind"] == kind and events[0]["robot"] == e["actor"]
            assert events[-1]["contract"] == events[0]["contract"]
            assert events[-1]["event"] == (
                "success"
                if e["is_success"]
                else "timeout"
                if e["reason"] == "timeout"
                else "failure"
            )
            assert abs(events[0]["time"]) < 1e-7
            assert abs(events[-1]["time"] - e["elapsed_seconds"]) < 1e-7
        report = audit_file(path, seed, count)
        passed = report["success_rate"] >= 0.95 and not any(
            report[k] for k in ("collisions", "field_exits", "own_goals", "premature_clearances")
        )
        report.update(
            mode=mode,
            checkpoint=checkpoint,
            orchestration_audit=True,
            runtime_hashes_verified=bool(freeze),
            gate_passed=passed,
            launch_sha256=sha(source.with_suffix(".launch.json")),
        )
        target = RELEASE / kind / (mode + "-audit.json")
        if target.exists():
            raise ValueError("Retain prior panel audit")
        write(target, report)
        reports[kind] = {k: v for k, v in report.items() if k not in ("cases", "checkpoint")}
    print(json.dumps(reports, indent=2))
    return all(r["gate_passed"] for r in reports.values())


def main():
    p = argparse.ArgumentParser()
    p.add_argument("source", type=Path)
    p.add_argument(
        "--mode", choices=("orchestration_development", "qualification", "demo"), required=True
    )
    a = p.parse_args()
    raise SystemExit(0 if panel(a.source.resolve(), a.mode) else 1)


if __name__ == "__main__":
    main()
