import numpy as np
import pytest

pytest.importorskip("torch")
from roboturk import pressure_audit


@pytest.mark.parametrize(
    "case,expected",
    [
        ("shield", True),
        ("stationary defender", False),
        ("ball exposed", False),
        ("only 19 frames", False),
        ("interrupted hold", False),
        ("different shooter", False),
    ],
)
def test_physical_shield_requires_approach_and_continuous_control(monkeypatch, case, expected):
    a = 24
    count = a + 20
    states = np.zeros((count, 22))
    robots = states[:, 4:].reshape(-1, 6, 3)
    robots[:] = [2, 2, 0]
    robots[:, 0] = [0, 0, 0]
    robots[:, 3] = [-0.40, 0, 0]
    states[:, :2] = [0.115, 0]
    if case != "stationary defender":
        robots[:a, 3, 0] -= (a - np.arange(a)) * 0.006
    if case == "ball exposed":
        robots[:, 3, 0] *= -1
    if case == "interrupted hold":
        states[a + 8, 0] = 0.22
    if case == "only 19 frames":
        states = states[:-1]
    chain = {"team": "blue", "scorer": int(case == "different shooter")}
    monkeypatch.setattr(
        pressure_audit,
        "lead_result",
        lambda *args: dict(qualified=True, chain=chain, receiver=0, contact=a),
    )
    p = dict(
        pressure_m=0.75, approach_mps=0.15, shield_m=0.85, shield_cosine=0.45, shield_samples=20
    )
    row = pressure_audit.pressure_result(states, {"events": []}, p)
    assert row["qualified"] is expected
    if expected:
        assert (row["shield_start"], row["shield_end"]) == (a, a + 19)
