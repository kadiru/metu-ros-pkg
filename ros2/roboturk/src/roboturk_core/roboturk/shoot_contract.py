"""Stateless goal-conditioned capture/carry/shoot observation and actuator contract."""

import numpy as np
from .dribble_contract import dribble_observation
from .model import Command, bounded

SHOOT_CONTRACT = "roboturk.capture_dribble_shoot.v1"
SHOT_SPEED = 3.5


def shoot_observation(world, key, intent, cfg):
    # Intent.target is the actual opponent goal center, not the dribble stopping point.
    return dribble_observation(world, key, intent, cfg)


def shoot_command(action, cfg):
    action = np.asarray(action, dtype=np.float32)
    if action.shape != (4,) or not np.all(np.isfinite(action)):
        raise ValueError("Expected four finite shooting actions")
    a = np.clip(action, -1, 1)
    fire = a[3] > 0
    # Skill-level actuator semantics: retain until firing; fixed calibrated-in-simulation request.
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
