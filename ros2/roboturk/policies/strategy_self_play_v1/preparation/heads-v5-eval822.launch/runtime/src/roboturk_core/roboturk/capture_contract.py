"""Body-relative chase/capture contract shared by training and deployed inference."""

import math
import numpy as np
from .model import Command, body_vector, bounded, possession

CAPTURE_CONTRACT = "roboturk.capture.v1"


def capture_observation(world, key, cfg):
    robot, ball = world.robot(key), world.ball
    bx, by = body_vector(ball.x - robot.x, ball.y - robot.y, robot.yaw)
    distance = max(1e-6, math.hypot(bx, by))
    bvx, bvy = body_vector(ball.vx, ball.vy, robot.yaw)
    rvx, rvy = body_vector(robot.vx, robot.vy, robot.yaw)
    result = np.asarray(
        [
            bx / 3,
            by / 3,
            bx / distance,
            by / distance,
            bvx / cfg.max_speed,
            bvy / cfg.max_speed,
            rvx / cfg.max_speed,
            rvy / cfg.max_speed,
            robot.omega / cfg.max_omega,
            np.clip((bx - cfg.possession_x) / 0.3, -5, 5),
            np.clip(by / 0.3, -5, 5),
            float(possession(world, key, cfg)),
        ],
        dtype=np.float32,
    )
    if not np.all(np.isfinite(result)):
        raise ValueError("Nonfinite capture observation")
    return result


def capture_command(action, cfg):
    action = np.asarray(action, dtype=np.float32)
    if action.shape != (4,) or not np.all(np.isfinite(action)):
        raise ValueError("Expected four finite capture actions")
    a = np.clip(action, -1, 1)
    return bounded(
        Command(
            float(a[0]) * cfg.max_speed,
            float(a[1]) * cfg.max_speed,
            float(a[2]) * cfg.max_omega,
            max(0.0, float(a[3])),
            0.0,
        ),
        cfg,
    )


def capture_action(command, cfg):
    c = bounded(command, cfg)
    return np.asarray(
        [c.vx / cfg.max_speed, c.vy / cfg.max_speed, c.omega / cfg.max_omega, c.dribbler],
        dtype=np.float32,
    )
