"""Inward-facing recovery from the playable carpet outside a touchline."""

import math

from . import field
from .carrier_motor import MOTOR, track_roller
from .five_roles import CFG
from .model import wrap
from .three_recipient import move

# Keep the field end inset, so the robot disk clears both goal frames.
END_LIMIT = field.LENGTH / 2 - 0.15
SIDE_LIMIT = field.WIDTH / 2 + field.WALL_MARGIN - CFG.robot_radius - 0.025


def brake_axis(position, request, limit, dt, *, acceleration=CFG.max_acceleration):
    """Reserve a reaction tick plus braking distance in each outward direction."""
    if request == 0:
        return 0.0
    sign = math.copysign(1.0, request)
    room = max(0.0, limit - sign * position)
    a = acceleration / math.sqrt(2)
    cap = math.sqrt((a * dt) ** 2 + 2 * a * room) - a * dt
    return sign * min(abs(request), cap)


def recovery_command(world, robot, role, sole):
    ball = world.ball
    if not ball.visible or abs(ball.y) > CFG.ball_exit_y or abs(ball.x) > CFG.ball_exit_x:
        return None
    owns = robot.key == sole
    if not owns and (sole is not None or role not in ("Chaser", "Receiver", "Presser", "Carrier")):
        return None
    if abs(ball.y) < field.WIDTH / 2 - (0.36 if owns else 0.18):
        return None
    side = math.copysign(1, ball.y)
    yaw = -side * math.pi / 2
    if owns:
        omega = max(-2.5, min(2.5, 5 * wrap(yaw - robot.yaw)))
        return track_roller(world, robot, (0.0, -side * 0.75), omega, MOTOR)
    # Round the ball on its flank before coming in from the board side.
    lead = min(0.35, math.hypot(robot.x - ball.x, robot.y - ball.y) / 1.2)
    bx = max(-END_LIMIT, min(END_LIMIT, ball.x + lead * ball.vx))
    by = min(SIDE_LIMIT, abs(ball.y) + CFG.possession_x)
    outward = side * robot.y - abs(ball.y)
    if outward < 0.08:
        flank = 1 if robot.x >= bx else -1
        if abs(bx + flank * 0.30) > END_LIMIT:
            flank = -flank
        tx = bx + flank * 0.30
        ty = side * (abs(ball.y) - 0.14 if abs(robot.x - bx) < 0.18 else by + 0.035)
    else:
        tx, ty = bx, side * by
    return move(
        robot, max(-END_LIMIT, min(END_LIMIT, tx)), max(-SIDE_LIMIT, min(SIDE_LIMIT, ty)), yaw, 1.05
    )
