"""Ball-side challenge for the 5v5 Presser with a fitted angle and release motion."""

from dataclasses import dataclass
import math

from .model import Command, body_vector, world_vector, wrap

ROBOT_GAP = 0.202


@dataclass(frozen=True)
class BallSideTackle:
    angle_deg: float
    mouth_offset: float
    draw_speed: float
    flank: float

    def __post_init__(self):
        if not (
            15 <= self.angle_deg <= 70
            and 0.09 <= self.mouth_offset <= 0.15
            and 0.08 <= self.draw_speed <= 0.35
            and 0 <= self.flank <= 1
        ):
            raise ValueError("Ball-side tackle profile outside the fitted motor range")

    def command(self, world, presser, carrier, observed, has_ball):
        ball = world.ball
        nx, ny = math.cos(carrier.yaw), math.sin(carrier.yaw)
        x, y = presser.x - carrier.x, presser.y - carrier.y
        side = math.copysign(1, nx * y - ny * x)
        cx, cy = observed[carrier.key]
        face = math.atan2(ball.y - presser.y, ball.x - presser.x)
        omega = 6 * wrap(face - presser.yaw)
        ball_depth = (ball.x - carrier.x) * nx + (ball.y - carrier.y) * ny
        mouth_distance = math.hypot(ball.x - presser.x, ball.y - presser.y)
        if has_ball and (mouth_distance < 0.127 or ball_depth > 0.122):
            anchor = carrier.yaw + side * math.radians(self.angle_deg) + math.pi
            omega = max(-4.2, min(4.2, 7 * wrap(anchor - presser.yaw)))
            # Drag across the carrier's narrower forward roller boundary.
            # Limit stretch in both directions of our own roller.
            mx, my = body_vector(
                0.047 * (nx - side * self.flank * ny),
                0.047 * (ny + side * self.flank * nx),
                presser.yaw,
            )
            mx = max(-0.030, min(0.030, mx))
            my = max(-0.040, min(0.040, my))
            err_x, err_y = body_vector(ball.x - presser.x, ball.y - presser.y, presser.yaw)
            ax, ay = world_vector(err_x - 0.115 + mx, err_y + my, presser.yaw)
            gain = min(7.0, self.draw_speed / max(0.001, math.hypot(ax, ay)))
            ox, oy = 0.115 * math.cos(presser.yaw), 0.115 * math.sin(presser.yaw)
            vx, vy = body_vector(
                ball.vx + gain * ax + omega * oy,
                ball.vy + gain * ay - omega * ox,
                presser.yaw,
            )
            return Command(vx, vy, omega, 1)

        direction = carrier.yaw + side * math.radians(self.angle_deg)
        goal_x = ball.x + self.mouth_offset * math.cos(direction)
        goal_y = ball.y + self.mouth_offset * math.sin(direction)
        dx, dy = goal_x - presser.x, goal_y - presser.y
        gain = min(4.2, 0.82 / max(0.01, math.hypot(dx, dy)))
        # If a challenger is level with or behind the carrier, go around its
        # ball-side hip before cutting across to the forward interception point.
        if nx * x + ny * y < 0.08 and math.hypot(x, y) < 0.42:
            dx, dy = 0.20 * nx - side * 0.24 * ny - x, 0.20 * ny + side * 0.24 * nx - y
        vx, vy = body_vector(cx + gain * dx, cy + gain * dy, presser.yaw)
        return Command(vx, vy, max(-5.0, min(5.0, omega)), 1)
