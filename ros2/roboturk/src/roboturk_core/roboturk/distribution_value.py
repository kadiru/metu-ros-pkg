"""Infer post-steal action values from the two candidate routes and carrier state."""

import numpy as np


def design(x, action):
    forward, side, pressure, aperture, yaw, speed = x[:6]
    state = [
        forward / 4,
        side / 2.8,
        min(1.5, pressure),
        min(0.7, aperture) * 2,
        yaw / 3.14,
        min(speed, 2) / 2,
    ]
    if action < 2:
        a, b = x[6:13], x[13:20]
        state.extend([min(a[0], 1), min(a[2], 0.7), min(b[0], 1), min(b[2], 0.7)])
    else:
        lane, space, shot, progress, length, turn, run = x[
            6 + 7 * (action - 2) : 13 + 7 * (action - 2)
        ]
        state.extend(
            [
                min(1, lane),
                min(space, 2) / 2,
                min(0.7, shot) * 2,
                progress / 3,
                min(length, 5) / 4,
                turn / 3.14,
                run,
            ]
        )
    v = np.asarray(state)
    return np.r_[1, v, v * v]


def decision(model, x):
    if "heads" not in model:
        from .takeaway import select

        return select(model, x)
    scores = [float(design(x, a) @ model["heads"][min(a, 2)]) for a in range(4)]
    return max(range(4), key=lambda a: scores[a])
