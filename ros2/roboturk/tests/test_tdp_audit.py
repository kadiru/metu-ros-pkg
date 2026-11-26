"""Reject invented success using retained native traces and deliberate mutations."""

from copy import deepcopy
import json
from pathlib import Path
import runpy
import pytest

ROOT = Path(__file__).resolve().parents[1]
AUDIT = runpy.run_path(str(ROOT / "scripts/audit_tdp.py"))["audit_episode"]
CASES = json.loads((ROOT / "tests/fixtures/tdp-native-baselines.json").read_text())


def test_unmodified_native_baselines_pass():
    assert all(AUDIT(e)["success"] for e in CASES.values())


def test_claimed_redirect_without_actual_kick_is_rejected():
    e = deepcopy(CASES["redirect"])
    e["events"] = [v for v in e["events"] if not (v["type"] == "kick" and v["robot"] == e["actor"])]
    e["actual_actor_kicks"] = 0
    with pytest.raises(AssertionError):
        AUDIT(e)


def test_fabricated_contact_time_is_rejected():
    e = deepcopy(CASES["redirect"])
    e["first_contact_time"] += 0.2
    with pytest.raises(AssertionError):
        AUDIT(e)


def test_fabricated_stable_reception_is_rejected():
    e = deepcopy(CASES["receive"])
    e["samples"][-1]["ball"][2] = 3
    with pytest.raises(AssertionError):
        AUDIT(e)


def test_unreported_field_excursion_is_rejected():
    e = deepcopy(CASES["clear"])
    actor = next(
        r
        for r in e["samples"][20]["robots"]
        if r["team"] == e["actor"]["team"] and r["id"] == e["actor"]["id"]
    )
    actor["pose"][1] = 3
    with pytest.raises(AssertionError):
        AUDIT(e)
