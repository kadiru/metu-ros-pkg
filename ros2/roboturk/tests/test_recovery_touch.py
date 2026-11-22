import numpy as np
import pytest

pytest.importorskip("torch")
from roboturk.touch_audit import first_loss
from roboturk.recovery_touch import touch_choice
from roboturk.model import Ball, Robot, RobotKey, World


@pytest.mark.parametrize("issue", ["clean", "slip", "dual", "restart", "shot"])
def test_count_every_break_before_the_actual_first_pass(issue):
    s = np.zeros((40, 22))
    robots = s[:, 4:].reshape(-1, 6, 3)
    robots[:, :, :2] = 2
    robots[:, 0] = [0, 0, 0]
    s[:, 0] = 0.115
    if issue == "slip":
        s[10, 1] = 0.046
    if issue == "dual":
        robots[10, 3] = [0, 0, 0]
    game = dict(events=[dict(event="kick", tick=4)], passes=[dict(tick=4, team="blue", passer=0)])
    if issue == "restart":
        game["events"].insert(0, dict(event="ball out", tick=2))
    if issue == "shot":
        game["passes"] = []
    s[16:, 0] = 1.0
    r = first_loss(s, game, dict(team="blue", winner=0, recovery=6))
    assert r["prepass_loss"] == (issue != "clean")
    assert r["first_break"] == (10 if issue in ("slip", "dual") else None)
    assert r["first_pass"] == (None if issue in ("restart", "shot") else 4)


def test_policy_responds_to_the_moving_ball_and_pressure_direction():
    actor = Robot(RobotKey("blue", 0), 0, 0)
    rival = Robot(RobotKey("yellow", 0), 0.22, 0)
    mate = Robot(RobotKey("blue", 1), 1, 0)
    p = dict(
        look=0.07,
        cruise=0.56,
        rush_weight=1.4,
        side_bias=0.55,
        side_cost=0.12,
        release_value=1.3,
        shield_bias=0.3,
    )

    def decision(ball, opponent):
        w = World(0, (actor, mate, opponent), ball)
        return touch_choice(w, actor, opponent, mate, p)[0]

    assert decision(Ball(0.115, 0), rival) == "release"
    assert decision(Ball(0.115, 0.025, 0, 1.3), rival) == "shield"
    assert (
        decision(Ball(0.115, 0.025, 0, 1.3), Robot(RobotKey("yellow", 0), 0.8, 0.25)) == "sidestep"
    )
