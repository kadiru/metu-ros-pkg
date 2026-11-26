import pytest

pytest.importorskip("torch")
from roboturk.model import RobotKey
from roboturk.takeaway import TakeawayObserver, select


def switches(runs):
    detector = TakeawayObserver()
    sample = 0
    takeaways = []
    for keys, count in runs:
        for _ in range(count):
            key = detector.step(keys, sample)
            if key:
                takeaways.append((sample, key))
            sample += 1
    return takeaways


def test_live_turnover_requires_two_confirmed_sides():
    blue, yellow = RobotKey("blue", 0), RobotKey("yellow", 1)
    assert switches([([blue], 19), ([], 5), ([yellow], 20)]) == []
    assert switches([([blue], 20), ([], 5), ([yellow], 20)]) == [(44, yellow)]


def test_contested_win_does_not_trigger_clean_takeaway_option():
    blue, yellow = RobotKey("blue", 2), RobotKey("yellow", 0)
    assert switches([([blue], 30), ([blue, yellow], 20), ([yellow], 20)]) == []
    assert switches([([blue], 30), ([blue, yellow], 19), ([yellow], 20)]) == [(68, yellow)]


def test_option_split_at_training_threshold():
    tree = dict(feature=1, threshold=1.0, left=dict(action=1), right=dict(action=4))
    assert select(tree, [0, 1.0]) == 1
    assert select(tree, [0, 2.0]) == 4
