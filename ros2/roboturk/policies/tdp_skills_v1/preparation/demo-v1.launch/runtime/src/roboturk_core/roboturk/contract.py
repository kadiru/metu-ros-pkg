"""One versioned observation/action ABI for training and live behavior inference."""

import numpy as np

from .model import Command, Config, Intent, RobotKey, Skill, World, body_vector, bounded, possession

CONTRACT_VERSION = "roboturk.skills.v1"
SKILLS = tuple(Skill)


def observation_size(cfg: Config) -> int:
    return 17 + len(SKILLS) + 12 * cfg.neighbors


def observe(world: World, key: RobotKey, intent: Intent, cfg: Config) -> np.ndarray:
    r = world.robot(key)
    b = world.ball
    vx, vy = body_vector(r.vx, r.vy, r.yaw)
    bx, by = body_vector(b.x - r.x, b.y - r.y, r.yaw)
    bvx, bvy = body_vector(b.vx - r.vx, b.vy - r.vy, r.yaw)
    tx, ty = body_vector(intent.target[0] - r.x, intent.target[1] - r.y, r.yaw)
    gx, gy = body_vector(world.direction(key.team), 0, r.yaw)
    # Position normalized by field length/width; body-frame directions and visibility retained.
    values = [
        vx / cfg.max_speed,
        vy / cfg.max_speed,
        r.omega / cfg.max_omega,
        bx / cfg.length if b.visible else 0,
        by / cfg.width if b.visible else 0,
        bvx / cfg.max_kick if b.visible else 0,
        bvy / cfg.max_kick if b.visible else 0,
        float(b.visible),
        tx / cfg.length,
        ty / cfg.width,
        gx,
        gy,
        r.x / cfg.length,
        r.y / cfg.width,
        np.sin(r.yaw),
        np.cos(r.yaw),
        float(possession(world, key, cfg)),
    ]
    values.extend(float(intent.skill == s) for s in SKILLS)
    for same_team in (True, False):
        neighbors = [
            n
            for n in world.robots
            if n.key != key and n.visible and (n.key.team == key.team) == same_team
        ]
        neighbors.sort(key=lambda n: ((n.x - r.x) ** 2 + (n.y - r.y) ** 2, n.key))
        for i in range(cfg.neighbors):
            if i >= len(neighbors):
                values.extend([0.0] * 6)
                continue
            n = neighbors[i]
            nx, ny = body_vector(n.x - r.x, n.y - r.y, r.yaw)
            nvx, nvy = body_vector(n.vx - r.vx, n.vy - r.vy, r.yaw)
            values.extend(
                [
                    nx / cfg.length,
                    ny / cfg.width,
                    nvx / (2 * cfg.max_speed),
                    nvy / (2 * cfg.max_speed),
                    1.0,
                    float(n.key == intent.receiver),
                ]
            )
    result = np.asarray(values, dtype=np.float32)
    if not np.all(np.isfinite(result)):
        raise ValueError("Nonfinite observation")
    return result


def decode_action(action, cfg: Config) -> Command:
    a = np.asarray(action, dtype=np.float32)
    if a.shape != (5,) or not np.all(np.isfinite(a)):
        raise ValueError("Expected five finite actions")
    a = np.clip(a, -1.0, 1.0)
    return bounded(
        Command(
            float(a[0]) * cfg.max_speed,
            float(a[1]) * cfg.max_speed,
            float(a[2]) * cfg.max_omega,
            max(0.0, float(a[3])),
            max(0.0, float(a[4])) * cfg.max_kick,
        ),
        cfg,
    )


def encode_action(command: Command, cfg: Config) -> np.ndarray:
    c = bounded(command, cfg)
    return np.asarray(
        [
            c.vx / cfg.max_speed,
            c.vy / cfg.max_speed,
            c.omega / cfg.max_omega,
            c.dribbler,
            c.kick / cfg.max_kick,
        ],
        dtype=np.float32,
    )
