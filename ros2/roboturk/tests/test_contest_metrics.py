import numpy as np
from roboturk.contest_metrics import measures


def test_opposing_touch_invalidates_recovery_delivery():
    states = np.zeros((140, 22))
    poses = states[:, 4:].reshape(-1, 6, 3)
    poses[:, :, 1] = 2
    poses[:60, 0, :] = [-0.115, 0, 0]
    poses[:30, 3, :] = [0.115, 0, np.pi]
    states[60:, 0] = 0.8
    poses[65:, 1, :] = [0.685, 0, 0]
    game = dict(
        seconds=1.4,
        events=[dict(event="kick", tick=15, team="blue", robot=0, speed=2.2)],
        passes=[
            dict(result="complete", team="blue", passer=0, receiver=1, tick=15, end=22, ball=[0, 0])
        ],
    )
    assert measures(states, game)["chains"] == 1
    poses[63, 4, :] = [0.685, 0, 0]
    assert measures(states, game)["chains"] == 0
