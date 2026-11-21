import json
from pathlib import Path

import numpy as np
import pytest

from roboturk.contest_metrics import measures
from roboturk.model import Ball, Robot, RobotKey


@pytest.fixture
def runtime():
    pytest.importorskip("torch", reason="The native policy runtime uses the RL extra.")
    from roboturk import jaw_recovery

    return jaw_recovery


def test_recovery_credit_ends_before_the_recipient_touches(runtime):
    learner = runtime.ReturnLearner()
    x = [1, 0, 0, 0, 0, 0, -1, 0]
    game = dict(
        seconds=8,
        challenger="blue",
        contest_metrics=dict(contests=[dict(winner=0, recovery=60, received=30)]),
        learned_actions=[
            dict(head="jaw", team="blue", tick=8, action=3, context=x),
            dict(head="jaw", team="yellow", tick=8, action=1, context=x),
            dict(head="jaw", team="blue", tick=20, action=7, context=x),
        ],
    )
    assert learner.update(game) == 6
    values = np.array(learner.policy()["q"]) @ x
    assert values[3] > 0 and np.count_nonzero(values) == 1


def test_context_depends_on_the_ball_in_both_dribblers(runtime):
    a = Robot(RobotKey("blue", 0), -0.115, 0, 0)
    b = Robot(RobotKey("yellow", 2), 0.115, 0, np.pi)
    center = runtime.context(a, b, Ball(0, 0), 0)
    edge = runtime.context(a, b, Ball(0, 0.02), 0)
    assert abs(center[2]) < 1e-8 and abs(center[4]) < 1e-8
    assert edge[2] > 0 and edge[4] < 0


def test_audited_open_play_transfers_in_core_environment():
    folder = (
        Path(__file__).resolve().parents[1] / "policies/contest_outlet_v2/qualification/trained"
    )
    game = json.loads((folder / "game-009.json").read_text())
    with np.load(folder / "trace-009.npz") as trace:
        audited = measures(trace["states"], game)
    assert audited == game["contest_metrics"]
    assert [(e["winner"], e["receiver"]) for e in audited["contests"] if "received" in e] == [
        (4, 3),
        (4, 3),
        (1, 0),
    ]
