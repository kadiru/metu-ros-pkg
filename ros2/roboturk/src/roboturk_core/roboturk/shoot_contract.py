"""Stateless goal-conditioned capture/carry/shoot observation and actuator contract."""

import numpy as np
from .dribble_contract import dribble_observation
from .model import Command, bounded, skill_limits

SHOOT_CONTRACT_V1 = "roboturk.capture_dribble_shoot.v1"
SHOOT_CONTRACT = "roboturk.capture_dribble_shoot.v2"
SHOT_SPEED = 3.5


def shoot_observation(world, key, intent, cfg):
    # Intent.target is the actual opponent goal center, not the dribble stopping point.
    original = dribble_observation(world, key, intent, cfg)
    # Make radial goal range explicit so firing need not infer a norm from rotated components.
    return np.concatenate([original, [np.linalg.norm(original[12:14])]]).astype(np.float32)


def shoot_command(action, cfg):
    cfg = skill_limits(cfg)
    action = np.asarray(action, dtype=np.float32)
    if action.shape != (4,) or not np.all(np.isfinite(action)):
        raise ValueError("Expected four finite shooting actions")
    a = np.clip(action, -1, 1)
    fire = a[3] > 0
    # Skill-level actuator semantics: retain until firing; fixed simulated kick request.
    return bounded(
        Command(
            float(a[0]) * cfg.max_speed,
            float(a[1]) * cfg.max_speed,
            float(a[2]) * cfg.max_omega,
            0.0 if fire else 1.0,
            SHOT_SPEED if fire else 0.0,
        ),
        cfg,
    )


def shoot_action(command, cfg):
    cfg = skill_limits(cfg)
    c = bounded(command, cfg)
    return np.asarray(
        [
            c.vx / cfg.max_speed,
            c.vy / cfg.max_speed,
            c.omega / cfg.max_omega,
            1.0 if c.kick > 0 else -1.0,
        ],
        dtype=np.float32,
    )
