"""Replay auditor rejects damaged evidence, independently of environment metrics."""

import copy
import importlib.util
from pathlib import Path

from test_strategy_options import Bank, Replay
from roboturk.strategy_options import StrategyMatch
import numpy as np

spec = importlib.util.spec_from_file_location(
    "audit_strategy", Path(__file__).resolve().parents[1] / "scripts/audit_strategy.py"
)
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def record():
    match = StrategyMatch(Replay(), Bank(), duration=0.1, trace=True)
    match.reset(6100000)
    actions = np.zeros((4, 17), np.float32)
    actions[:, 7] = 0.8
    actions[:, 12] = 1
    while not match.finished:
        match.step(actions)
    return match.result()


def test_missing_physics_and_fabricated_success_fail():
    original = record()
    assert audit.audit_match(original)["passed"]
    bad = copy.deepcopy(original)
    bad["physics_trace"].pop()
    assert not audit.audit_match(bad)["passed"]
    bad = copy.deepcopy(original)
    bad["option_events"].append({"type": "receive_stop", "step": 4, "robot": "blue0"})
    assert "skill event geometry/timing" in audit.audit_match(bad)["errors"]


def test_hidden_collision_and_inference_mismatch_fail():
    bad = record()
    bad["physics_trace"][2][11:13] = bad["physics_trace"][2][5:7]
    assert "contacts" in audit.audit_match(bad)["errors"]
    bad = record()
    bad["motor_calls"]["receive"] -= 1
    assert "motor inference counts" in audit.audit_match(bad)["errors"]
