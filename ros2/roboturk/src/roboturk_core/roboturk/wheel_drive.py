"""RoboTurk's historical four-wheel geometry and an ideal actuator envelope."""

from dataclasses import replace
from math import cos, isfinite, radians, sin

import numpy as np

from .model import body_vector


WHEEL_ANGLES_DEG = (60, 135, -135, -60)
WHEEL_RADIUS_M = 0.025
WHEEL_OFFSET_M = 0.085
DRIBBLER_WIDTH_M = 0.09
BEAM_X_M = 0.102
BEAM_HEIGHT_M = 0.022

# Positive shaft direction follows SSLPlayer::move in the original RoboTurk stack.
_TO_SHAFT = (
    np.array(
        [(sin(radians(theta)), -cos(radians(theta)), -WHEEL_OFFSET_M) for theta in WHEEL_ANGLES_DEG]
    )
    / WHEEL_RADIUS_M
)
_TO_BODY = np.linalg.pinv(_TO_SHAFT)


def shafts_for_twist(forward, left, yaw_rate):
    return tuple(_TO_SHAFT @ (forward, left, yaw_rate))


def twist_for_shafts(speeds):
    return tuple(_TO_BODY @ speeds)


def limit_by_shafts(command, max_rad_s):
    """Scale the complete twist when the first motor reaches its shaft limit."""
    if not isfinite(max_rad_s) or max_rad_s <= 0:
        raise ValueError("Shaft speed limit must be finite and positive")
    peak = max(map(abs, shafts_for_twist(command.vx, command.vy, command.omega)))
    if peak <= max_rad_s:
        return command
    scale = max_rad_s / peak
    return replace(
        command, vx=scale * command.vx, vy=scale * command.vy, omega=scale * command.omega
    )


def beam_broken(robot, ball, ball_radius):
    """Geometry proxy for a break beam across the roller ends, in the body frame."""
    if not (robot.visible and ball.visible):
        return False
    x, y = body_vector(ball.x - robot.x, ball.y - robot.y, robot.yaw)
    end_gap = max(0.0, abs(y) - DRIBBLER_WIDTH_M / 2)
    return (x - BEAM_X_M) ** 2 + (ball.z - BEAM_HEIGHT_M) ** 2 + end_gap**2 <= ball_radius**2
