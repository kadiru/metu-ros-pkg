"""Synthetic tampering checks for the release auditor, not simulation evidence."""

from copy import deepcopy
from importlib.util import module_from_spec, spec_from_file_location
import json
from pathlib import Path
import pytest


@pytest.fixture
def evidence(tmp_path, monkeypatch):
    root = Path(__file__).resolve().parents[1]
    spec = spec_from_file_location("selfplay_audit", root / "scripts/audit_self_play.py")
    audit = module_from_spec(spec)
    spec.loader.exec_module(audit)
    monkeypatch.setattr(audit, "ROOT", tmp_path)
    monkeypatch.setattr(audit, "SPLITS", {"final": (2300000, 2)})
    release = tmp_path / "release"
    release.mkdir()
    models = {}
    for role in audit.ROLES:
        path = tmp_path / (role + ".zip")
        path.write_bytes(("synthetic fixture " + role).encode())
        path.with_suffix(".json").write_text(
            json.dumps({"config": {"length": 9, "goal_width": 1, "ball_radius": 0.0215}})
        )
        models[role] = {
            "path": path.name,
            "sha256": audit.digest(path),
            "manifest_sha256": audit.digest(path.with_suffix(".json")),
        }
    selection = release / "selection.json"
    selection.write_text(json.dumps({"artifacts": []}))
    freeze = {
        "created_at": "2026-01-01T00:00:00-08:00",
        "models": {"final": models},
        "runtime_sha256": {},
        "selection_sha256": audit.digest(selection),
    }
    audit.write(release / "freeze.json", freeze)
    old = json.loads((root / "policies/learned_opponents_play_v1/qualification.json").read_text())
    result = {
        "models": models,
        "backend": "webots_unit_fixture",
        "opponent_policy_fallback": False,
        "inference_play": "PassReceiveScorePlay",
        "successes": 2,
        "episodes": deepcopy(old["episodes"][:2]),
    }
    for i, e in enumerate(result["episodes"]):
        e["seed"] = 2300000 + i
        for record in (e["transfer"], e["shooting"]):
            record.update(opponent="mobile_learned_defender_v1", keeper="mobile_learned_keeper_v1")
            for role in ("defender", "keeper"):
                record[role + "_checkpoint_sha256"] = models[role]["sha256"]
                record[role + "_outside_checks"] = 0
                record[role + "_x_excursion"] = 0.5
                record[role + "_y_excursion"] = 0.3
    path = release / "result.json"
    audit.write(path, result)
    return audit, release, path, result


def test_auditor_accepts_consistent_events_and_counts(evidence):
    audit, release, path, _ = evidence
    report = audit.audit(release, path, "final")
    assert report["passes_gate"] and report["counts"]["goals"] == 2


@pytest.mark.parametrize(
    "change", ["missing_seed", "goal_geometry", "opponent_disabled", "missing_contact", "collision"]
)
def test_auditor_rejects_false_success_or_inactive_opponent(evidence, change):
    audit, release, path, result = evidence
    e = result["episodes"][0]
    if change == "missing_seed":
        e["seed"] += 1
    elif change == "goal_geometry":
        e["shooting"]["ball_final"][0] = 0
    elif change == "opponent_disabled":
        e["shooting"]["keeper_policy_calls"] -= 1
    elif change == "missing_contact":
        e["reason"], e["is_success"] = "keeper_save", False
        result["successes"] = 1
    elif change == "collision":
        e["reason"], e["is_success"] = "defender_collision", False
        e["shooting"]["defender_robot_contacts"] = 1
        result["successes"] = 1
    audit.write(path, result)
    if change == "collision":
        assert not audit.audit(release, path, "final")["passes_gate"]
    else:
        with pytest.raises(ValueError):
            audit.audit(release, path, "final")


def test_auditor_rejects_changed_frozen_model(evidence):
    audit, release, path, _ = evidence
    (audit.ROOT / "keeper.zip").write_bytes(b"changed")
    with pytest.raises(ValueError, match="Model changed"):
        audit.audit(release, path, "final")
