"""Tamper native development records to check audit rejection, not performance."""

from copy import deepcopy
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import json
import pytest


@pytest.fixture
def evidence():
    root = Path(__file__).resolve().parents[1]
    spec = spec_from_file_location("decision_audit", root / "scripts/audit_decision_release.py")
    audit = module_from_spec(spec)
    spec.loader.exec_module(audit)
    data = json.loads(
        (root / "policies/decision_self_play_v1/development/teacher-corrected.json").read_text()
    )
    cfg = {"length": 9.0, "goal_width": 1.0, "ball_radius": 0.0215}
    return audit, deepcopy(data), cfg


def test_native_physical_event_records_are_consistent(evidence):
    audit, data, cfg = evidence
    for e in data["episodes"]:
        audit.validate_episode(e, cfg, data["models"])


@pytest.mark.parametrize(
    "change",
    [
        "crossing",
        "posts",
        "opponent_calls",
        "manager_calls",
        "decision_owner",
        "decision_option",
        "reception",
    ],
)
def test_tampered_events_and_decisions_rejected(evidence, change):
    audit, data, cfg = evidence
    e = next(e for e in data["episodes"] if e["reason"] == "goal" and e["stable_receptions"])
    if change == "crossing":
        e["goal_crossing_segment"][1][0] = 0
    elif change == "posts":
        e["goal_crossing_y"] = 1
    elif change == "opponent_calls":
        e["keeper_policy_calls"] -= 1
    elif change == "manager_calls":
        e["manager_calls"] -= 1
    elif change == "decision_owner":
        e["events"][-1]["decision_owner"] = 1 - e["events"][-1]["owner"]
    elif change == "decision_option":
        e["events"][-1]["type"] = "dribble"
    elif change == "reception":
        e["reception_events"][0]["dwell"] = 0
    with pytest.raises(ValueError):
        audit.validate_episode(e, cfg, data["models"])


def test_contact_is_required_for_a_defensive_win(evidence):
    audit, data, cfg = evidence
    e = next(e for e in data["episodes"] if e["reason"] == "keeper_save")
    e["keeper_ball_contacts"] = 0
    with pytest.raises(ValueError, match="contact"):
        audit.validate_episode(e, cfg, data["models"])


def test_invalid_shots_remain_failures_instead_of_disappearing(evidence):
    audit, data, cfg = evidence
    e = next(e for e in data["episodes"] if e["reason"] == "goal")
    e["events"][-1]["goal_distance"] = 3
    e["reason"] = "invalid_kick"
    e["is_success"] = False
    goal, stop, _, _ = audit.validate_episode(e, cfg, data["models"])
    assert not goal and not stop


@pytest.mark.parametrize("tamper", ["missing_gate", "unrequested_target_change"])
def test_learned_target_memory_evidence_rejects_tampering(evidence, tamper):
    audit, data, cfg = evidence
    root = Path(__file__).resolve().parents[1]
    raw = json.loads(
        (root / "policies/decision_self_play_v1/artifacts/prepared-v3-development.json").read_text()
    )
    e = deepcopy(raw["episodes"][0])
    models = {r: {"sha256": e[r + "_checkpoint_sha256"]} for r in ("defender", "keeper")}
    audit.validate_episode(e, cfg, models, require_gate=True)
    if tamper == "missing_gate":
        del e["decisions"][0]["receive_update"]
    else:
        t = next(t for t in e["decisions"][1:] if not t["receive_update"])
        t["receive"][0] += 0.1
    with pytest.raises(ValueError, match="target update|without learned update"):
        audit.validate_episode(e, cfg, models, require_gate=True)
