"""Centralized two-robot pass/reception contract, with explicit play memory."""

import math
import numpy as np
from .capture_contract import capture_observation
from .shoot_contract import shoot_observation
from .model import Command, Intent, Skill, bounded, skill_limits, body_vector

TEAM_CONTRACT_V1 = "roboturk.pass_receive.v1"
TEAM_CONTRACT = "roboturk.pass_receive.v2"
PASS_SPEED = 1.8


RECEIVER_CONTRACT = "roboturk.pass_receive.carpet_receiver.v1"
RECEIVER_RANGE = 0.65  # Same local approach range as the transfer demonstration controller.


def receiver_features(world, receiver, cfg):
    r, b = world.robot(receiver), world.ball
    bx, by = body_vector(b.x - r.x, b.y - r.y, r.yaw)
    if math.hypot(bx, by) >= RECEIVER_RANGE:
        return None
    vx, vy = body_vector(b.vx, b.vy, r.yaw)
    angle = math.atan2(by, bx)
    facing = max(0.0, math.cos(angle)) ** 3
    return np.array(
        [1, facing * (bx - cfg.possession_x), facing * by, facing * vx, facing * vy, angle]
    )


def team_observation(world, passer, receiver, state, cfg):
    p, r = world.robot(passer), world.robot(receiver)
    target = Intent(Skill.PASS, (r.x, r.y), receiver)
    observation = np.concatenate(
        [
            shoot_observation(world, passer, target, cfg),
            capture_observation(world, receiver, cfg),
            [
                math.cos(r.yaw - p.yaw),
                math.sin(r.yaw - p.yaw),
                float(state.passed),
                min(1.0, state.passer_dwell / 20),
                min(1.0, state.receiver_dwell / 20),
            ],
        ]
    ).astype(np.float32)
    if not np.all(np.isfinite(observation)):
        raise ValueError("Nonfinite team observation")
    return augment_angles(observation)


def augment_angles(observation):
    """Add signed aiming errors; usable on original training data, without labels."""
    x = np.asarray(observation, dtype=np.float32)
    return np.concatenate(
        [
            x,
            np.arctan2(x[..., 15:16], x[..., 14:15]) / math.pi,
            np.arctan2(x[..., 20:21], x[..., 19:20]) / math.pi,
        ],
        axis=-1,
    ).astype(np.float32)


def team_commands(action, passer, receiver, passed, cfg):
    cfg = skill_limits(cfg)
    a = np.asarray(action, dtype=np.float32)
    if a.shape != (7,) or not np.all(np.isfinite(a)):
        raise ValueError("Expected seven finite team actions")
    a = np.clip(a, -1, 1)
    fire = a[3] > 0
    # Explicit play/actuator semantics: passer holds after release; receiver cannot kick.
    return {
        passer: Command()
        if passed
        else bounded(
            Command(
                float(a[0]) * cfg.max_speed,
                float(a[1]) * cfg.max_speed,
                float(a[2]) * cfg.max_omega,
                0.0 if fire else 1.0,
                PASS_SPEED if fire else 0.0,
            ),
            cfg,
        ),
        receiver: bounded(
            Command(
                float(a[4]) * cfg.max_speed,
                float(a[5]) * cfg.max_speed,
                float(a[6]) * cfg.max_omega,
                1.0,
            ),
            cfg,
        ),
    }


def team_action(commands, passer, receiver, cfg):
    cfg = skill_limits(cfg)
    p, r = commands[passer], commands[receiver]
    return np.asarray(
        [
            p.vx / cfg.max_speed,
            p.vy / cfg.max_speed,
            p.omega / cfg.max_omega,
            1.0 if p.kick else -1.0,
            r.vx / cfg.max_speed,
            r.vy / cfg.max_speed,
            r.omega / cfg.max_omega,
        ],
        np.float32,
    )
