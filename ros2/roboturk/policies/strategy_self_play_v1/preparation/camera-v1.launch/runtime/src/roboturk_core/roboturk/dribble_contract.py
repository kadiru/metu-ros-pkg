"""Goal-conditioned capture/carry contract; identical in training and inference."""

import math
import numpy as np
from .capture_contract import capture_observation
from .model import body_vector

DRIBBLE_CONTRACT = "roboturk.capture_dribble.v1"


def dribble_observation(world, key, intent, cfg):
    r, b = world.robot(key), world.ball
    gx, gy = body_vector(intent.target[0] - b.x, intent.target[1] - b.y, r.yaw)
    distance = max(1e-6, math.hypot(gx, gy))
    observation = np.concatenate(
        [capture_observation(world, key, cfg), [gx / 5, gy / 5, gx / distance, gy / distance]]
    ).astype(np.float32)
    if not np.all(np.isfinite(observation)):
        raise ValueError("Nonfinite dribble observation")
    return observation
