"""Evidence audits must reject inflated scores and receptions after interceptions."""

import copy
from pathlib import Path
import runpy
import pytest

AUDIT = runpy.run_path(str(Path(__file__).parents[1] / "scripts/audit_continuous_release.py"))


def evidence():
    return {
        "metric_version": 2,
        "steps": 60,
        "duration": 1.2,
        "attack_sign": 1,
        "calls": {k: 60 for k in ("blue0", "blue1", "yellow0", "yellow1")},
        "mode_ticks": {"blue/recover": 120, "yellow/recover": 120},
        "physics_checks": 240,
        "contact_checks": 0,
        "contact_fraction": 0,
        "backend_resets": 1,
        "referee_restarts": 0,
        "possession_seconds": {"blue": 0.5, "yellow": 0},
        "contested_seconds": 0,
        "score": {"blue": 0, "yellow": 0},
        "passes": {},
        "shots": {},
        "receptions": {},
        "acquisitions": {},
        "turnovers": {},
        "recoveries": {},
        "events": [],
        "trace": [
            {
                "step": 60,
                "ball": [0, 0, 0, 0],
                "robots": [
                    [k, i, 0, 0, 0, 0]
                    for i, k in enumerate(("blue0", "blue1", "yellow0", "yellow1"))
                ],
            }
        ],
    }


def test_independent_goal_and_reset_audit_rejects_false_credit():
    r = evidence()
    r["score"]["blue"] = 1
    r["events"] = [
        {
            "type": "goal",
            "team": "blue",
            "step": 60,
            "direction": 1,
            "crossing_y": 0.1,
            "segment": [[4.51, 0.1], [4.53, 0.1]],
        }
    ]
    assert AUDIT["audit_match"](r) == []
    bad = copy.deepcopy(r)
    bad["events"][0]["segment"][-1][0] = 4.52
    with pytest.raises(AssertionError):
        AUDIT["audit_match"](bad)
    bad = copy.deepcopy(r)
    bad["backend_resets"] = 2
    with pytest.raises(AssertionError):
        AUDIT["audit_match"](bad)


def test_reception_audit_rejects_intervening_opponent_control():
    r = evidence()
    r["events"] = [
        {
            "type": "pass",
            "step": 1,
            "team": "blue",
            "robot": "blue0",
            "held": True,
            "mode": "pass",
            "speed": 2.2,
            "ball": [0, 0],
        },
        {
            "type": "acquisition",
            "step": 20,
            "team": "yellow",
            "robot": "yellow0",
            "previous_robot": None,
            "dwell": 15,
            "after_loose_ball": False,
            "ball": [0.6, 0],
        },
        {
            "type": "turnover",
            "step": 40,
            "team": "blue",
            "robot": "blue1",
            "previous_robot": "yellow0",
            "dwell": 15,
            "after_loose_ball": True,
            "ball": [1, 0],
        },
    ]
    r.update(
        passes={"blue": 1},
        acquisitions={"blue": 1, "yellow": 1},
        turnovers={"blue": 1},
        recoveries={"blue": 1},
    )
    AUDIT["audit_match"](r)
    r["events"].append(
        {
            "type": "reception",
            "step": 40,
            "pass_step": 1,
            "team": "blue",
            "robot": "blue1",
            "dwell": 15,
            "displacement": 1,
            "ball": [1, 0],
        }
    )
    r["receptions"] = {"blue": 1}
    with pytest.raises(AssertionError):
        AUDIT["audit_match"](r)
