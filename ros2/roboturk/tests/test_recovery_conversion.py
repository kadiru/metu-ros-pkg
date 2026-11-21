import numpy as np
from roboturk.recovery_conversion import conversions


def sequence():
    s = np.zeros((180, 22))
    robots = s[:, 4:].reshape(-1, 6, 3)
    robots[:, :, :2] = [2, 2]
    robots[:, 0] = [-0.115, 0, 0]
    robots[:24, 3] = [0.115, 0, np.pi]
    s[81:, 0] = np.linspace(0.01, 4.53, len(s) - 81)
    g = dict(
        seed=2,
        seconds=1.8,
        passes=[],
        options=[],
        events=[
            dict(tick=20, event="kick", team="blue", robot=0, speed=4),
            dict(tick=44, event="goal", team="yellow"),
        ],
    )
    return s, g


def test_goal_needs_an_unbroken_possession_and_opponent_goal_line():
    s, g = sequence()
    assert conversions(s, g)[0]["goal"] == 179
    s[63, 13:16] = [0.115, 0, np.pi]
    assert "goal" not in conversions(s, g)[0]
    s, g = sequence()
    s[-1, 0] = -4.53
    assert "goal" not in conversions(s, g)[0]


def test_restart_and_unintended_kick_invalidate_credit():
    for event in (
        dict(tick=15, event="ball out", team="blue"),
        dict(tick=15, event="kick", team="yellow", robot=2, speed=2.2),
    ):
        s, g = sequence()
        g["events"].insert(0, event)
        assert "goal" not in conversions(s, g)[0]
