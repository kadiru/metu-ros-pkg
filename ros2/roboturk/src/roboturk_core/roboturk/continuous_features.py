"""Measured local geometry at useful scales; no behavior, target or command rules."""

import numpy as np


def augment(base):
    """Append29 deterministic features to the original88-observation contract.

    Both scalar observations and recorded batches use exactly the same transform.
    Relative velocities, distances, bearings and fixed field/goal geometry are
    measurements; the policy still supplies every behavior and motor command.
    """
    x = np.asarray(base, dtype=np.float32)
    if x.shape[-1] != 88:
        raise ValueError("Expected88 base observations")
    c, s = x[..., 4], x[..., 5]
    fields = []
    for i in range(3):
        j, b = 9 * (i + 1), 52 + 4 * i
        dx, dy = 5 * x[..., b], 5 * x[..., b + 1]
        d = np.maximum(np.hypot(dx, dy), 1e-6)
        vx, vy = 2 * (x[..., j + 2] - x[..., 2]), 2 * (x[..., j + 3] - x[..., 3])
        fields += [
            np.clip(dx / 0.4, -2, 2),
            np.clip(dy / 0.4, -2, 2),
            dx / d,
            dy / d,
            (c * vx + s * vy) / 2,
            (-s * vx + c * vy) / 2,
            np.clip((d - 0.23) / 0.4, -2, 2),
        ]
    px, py = 4.5 * x[..., 0], 3 * x[..., 1]
    fields += [np.clip(v / 0.4, -2, 2) for v in (4.2 - px, 4.2 + px, 2.65 - py, 2.65 + py)]
    fields += [np.clip(x[..., i] * np.pi * 10, -2, 2) for i in (55, 67, 71, 75)]
    return np.concatenate([x, np.stack(fields, axis=-1)], axis=-1).astype(np.float32)
