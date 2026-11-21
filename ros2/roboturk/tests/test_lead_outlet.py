import pytest
import numpy as np

pytest.importorskip("torch")
from roboturk import lead_audit
from roboturk.lead_outlet import margin, rendezvous
from roboturk.model import Ball, Robot, RobotKey, World


def test_reactive_defender_closes_a_previously_open_pass():
    ball = Ball(0, 0)
    guard = Robot(RobotKey("yellow", 0), 1, 0.56)
    p = dict(roll=1.6, forecast=0.5)
    assert margin(ball, (1.6, 0), [guard], p) > 0.5
    assert margin(ball, (1.6, 0), [guard], p | dict(reach=1)) < 0.15


def test_rendezvous_respects_field_and_roller_offset():
    m = Robot(RobotKey("blue", 1), 3.96, 2.50)
    w = World(0, (m,), Ball(3, 1.1))
    target, center = rendezvous(
        w, m, (0, 1), dict(roll=1.6, run_speed=0.7, horizon=1, min_lead=0.3)
    )
    assert center[1] == pytest.approx(2.58)
    assert np.linalg.norm(np.subtract(center, target)) == pytest.approx(0.115)


@pytest.mark.parametrize("moving,expected", [(True, True), (False, False)])
def test_credit_requires_motion_at_release_and_ahead_of_contact(monkeypatch, moving, expected):
    base = dict(
        qualified=True,
        chain={"team": "blue"},
        kick=10,
        end=31,
        receiver=1,
        pass_result="complete",
        outcome="pass goal",
    )
    monkeypatch.setattr(lead_audit, "opening_result", lambda *args: base.copy())
    states = np.zeros((160, 22))
    robot = states[:, 7:10]
    robot[:, 0] = 1.115
    robot[:, 2] = np.pi
    robot[:, 1] = np.minimum(np.arange(160) * 0.005, 0.49) if moving else 0.49
    states[98:, :2] = [1, 0.49]
    plan = {"target": [1, 0.49], "runner": 1}
    protocol = dict(
        lead_distance_m=0.15, run_progress_m=0.12, run_speed_mps=0.12, target_tolerance_m=0.32
    )
    row = lead_audit.lead_result(states, {"plans": {"10": plan}}, protocol)
    assert row["qualified"] == expected
