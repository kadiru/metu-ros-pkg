"""A lofted outlet over a blocked flat passing lane."""

from dataclasses import dataclass
import math

from .model import GRAVITY, ROBOT_HEIGHT, launch_velocity

ELEVATION = math.radians(38)


@dataclass(frozen=True)
class ChipPass:
    speed: float
    elevation: float
    landing: tuple[float, float]


def chip_pass(world, passer, receiver, target, cfg):
    """Return a chip only when its flight clears the robots in the flat lane."""
    ball = world.ball
    dx, dy = target[0] - ball.x, target[1] - ball.y
    distance = math.hypot(dx, dy)
    if not 1.35 <= distance <= 3.8:
        return None
    ux, uy = dx / distance, dy / distance
    travel = distance - 0.55
    landing = ball.x + ux * travel, ball.y + uy * travel
    if abs(landing[0]) > cfg.length / 2 - 0.45 or abs(landing[1]) > cfg.width / 2 - 0.45:
        return None
    speed = math.sqrt(GRAVITY * travel / math.sin(2 * ELEVATION))
    if speed > cfg.max_kick:
        return None
    planar, _, up = launch_velocity(speed, 0, ELEVATION)
    blocked = False
    for robot in world.robots:
        if robot.key in (passer, receiver) or not robot.visible:
            continue
        x, y = robot.x - ball.x, robot.y - ball.y
        along = x * ux + y * uy
        if not 0 < along < distance:
            continue
        t = along / planar
        ahead = along + t * (robot.vx * ux + robot.vy * uy)
        lateral = (y + t * robot.vy) * ux - (x + t * robot.vx) * uy
        if abs(lateral) > cfg.robot_radius + cfg.ball_radius + 0.06:
            continue
        blocked = blocked or robot.key.team != passer.team
        # Check the near and far edges of the obstacle across the parabola.
        margin = cfg.robot_radius + cfg.ball_radius + 0.04
        for edge in (ahead - margin, ahead + margin):
            at = edge / planar
            z = ball.z + up * at - GRAVITY * at**2 / 2
            if not 0 < edge < travel or z < ROBOT_HEIGHT + cfg.ball_radius + 0.025:
                return None
    return ChipPass(speed, ELEVATION, landing) if blocked else None
