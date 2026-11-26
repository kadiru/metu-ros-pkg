import numpy as np
import pytest

from roboturk.open_play_audit import recoveries


def match():
    states = np.zeros((100, 22))
    robots = states[:, 4:].reshape(-1, 6, 3)
    robots[:, :, 0] = 2
    robots[:22, 3, 0] = -0.115
    robots[30:70, 1, 0] = -0.115
    states[-1, 0] = 4.55
    game = dict(
        seed=2,
        seconds=1,
        passes=[],
        events=[
            dict(tick=0, event="start"),
            dict(tick=18, event="kick", robot=1, team="blue", speed=4.0),
            dict(tick=24, event="goal"),
        ],
    )
    return states, game


def test_control_changes_require_a_sustained_opponent_and_an_unbroken_finish():
    states, game = match()
    (row,) = recoveries(states, game)
    assert (row["sample"], row["team"], row["scorer"], row["kind"]) == (49, "blue", 1, "takeaway")
    states[67, 4 + 3 * 3] = -0.115
    assert recoveries(states, game)[0]["outcome"] == "opponent touch"


@pytest.mark.parametrize("change", ["restart", "brief_touch", "teammate"])
def test_restarts_fleeting_contacts_and_teammate_handoffs_do_not_count(change):
    states, game = match()
    if change == "restart":
        game["events"].insert(1, dict(tick=6, event="ball out"))
    elif change == "brief_touch":
        states[19:22, 4 + 3 * 3] = 2
    else:
        states[:22, 4 + 3 * 3] = 2
        states[:22, 4] = -0.115
    assert recoveries(states, game) == []


def test_contest_and_takeaway_at_the_same_sample_are_one_recovery():
    states, game = match()
    states[22:42, 4 + 3 * 3] = -0.115
    states[22:42, 4 + 3 * 1] = -0.115
    (row,) = recoveries(states, game)
    assert row["kind"] == "contest" and row["sample"] == 61 and row["outcome"] == "goal"
