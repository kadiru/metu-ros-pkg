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


def test_opening_choice_changes_with_geometry_and_credits_the_first_possession():
    from roboturk.attack_opening import fit, training_example, values

    contexts = np.zeros((3, 17))
    contexts[:, 3] = [0, 0.5, 1.0]
    examples = [
        dict(action=act, x=x.tolist(), value=5.0 if i == act else -1.0)
        for i, x in enumerate(contexts)
        for act in range(3)
    ]
    policy = fit(examples, neighbors=1)
    assert [int(np.argmax(values(policy, x))) for x in contexts] == [0, 1, 2]
    game = dict(seed=2, conversions=[dict(start=0), dict(recovery=80, goal=240)], options=[])
    assert training_example(game) is None
