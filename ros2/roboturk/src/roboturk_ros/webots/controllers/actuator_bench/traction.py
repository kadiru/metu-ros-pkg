"""Reduced-order carpet contact for four motorized omniwheels.

Webots supports the weight on the wheel collision cylinders. This model supplies
traction along each wheel's driven tangent and small drag along its free rollers.
"""

from math import atan2, cos, hypot, radians, sin

from roboturk.model import body_vector, world_vector
from roboturk.wheel_drive import WHEEL_ANGLES_DEG, WHEEL_OFFSET_M, WHEEL_RADIUS_M


WHEELS = tuple((cos(radians(angle)), sin(radians(angle))) for angle in WHEEL_ANGLES_DEG)
TRACTION_N_PER_M_S = 30.0
FREE_ROLLER_DRAG_N_PER_M_S = 0.55
MAX_DRIVE_FORCE_N = 5.8
CONTACT_SLOP_M = 0.003


def apply_traction(chassis, wheel_rad_s):
    orientation = chassis.getOrientation()
    height = chassis.getPosition()[2]
    vx, vy, _, _, _, yaw_rate = chassis.getVelocity()
    forward, left = body_vector(vx, vy, atan2(orientation[3], orientation[0]))
    for (rx, ry), wheel_speed in zip(WHEELS, wheel_rad_s, strict=True):
        x, y = WHEEL_OFFSET_M * rx, WHEEL_OFFSET_M * ry
        wheel_z = height + orientation[6] * x + orientation[7] * y + orientation[8] * WHEEL_RADIUS_M
        if wheel_z > WHEEL_RADIUS_M + CONTACT_SLOP_M:
            continue
        rolling_speed = ry * forward - rx * left - WHEEL_OFFSET_M * yaw_rate
        slip = wheel_speed * WHEEL_RADIUS_M - rolling_speed
        pull = max(-MAX_DRIVE_FORCE_N, min(MAX_DRIVE_FORCE_N, TRACTION_N_PER_M_S * slip))
        drag = -FREE_ROLLER_DRAG_N_PER_M_S * (rx * forward + ry * left)
        chassis.addForceWithOffset(
            [pull * ry + drag * rx, -pull * rx + drag * ry, 0.0],
            [x, y, 0.0],
            True,
        )


# A narrow compliant contact patch supplements the rigid Webots roller.
HOLD_X_M = 0.123
CONTACT_HALF_LENGTH_M = 0.019
MAX_ROLLER_FORCE_N = 2.8


def apply_roller_contact(chassis, ball, roller_rad_s):
    if roller_rad_s < 50:
        return
    x, y, _ = chassis.getPosition()
    bx, by, bz = ball.getPosition()
    orientation = chassis.getOrientation()
    yaw = atan2(orientation[3], orientation[0])
    forward, left = body_vector(bx - x, by - y, yaw)
    if abs(forward - HOLD_X_M) > CONTACT_HALF_LENGTH_M or abs(left) > 0.041 or bz > 0.034:
        return
    vx, vy, _, _, _, yaw_rate = chassis.getVelocity()
    bvx, bvy, *_ = ball.getVelocity()
    dx, dy = body_vector(bvx - vx, bvy - vy, yaw)
    grip = min(1, roller_rad_s / 200)
    fx = grip * (95 * (HOLD_X_M - forward) - 3.7 * (dx + yaw_rate * left))
    fy = grip * (-14 * left - 0.9 * (dy - yaw_rate * forward))
    scale = min(1, MAX_ROLLER_FORCE_N / max(1e-9, hypot(fx, fy)))
    fx, fy = scale * fx, scale * fy
    ball.addForce([*world_vector(fx, fy, yaw), 0], False)
    chassis.addForceWithOffset([-fx, -fy, 0], [forward, left, 0.022], True)
