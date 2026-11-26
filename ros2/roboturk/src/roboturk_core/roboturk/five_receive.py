"""Cushion and open up for a shot during a 5v5 pass reception."""

from dataclasses import dataclass
import math

from .carpet import ROLL_DECEL
from .carrier_motor import track_roller
from .counterpass import shot_window
from .five_roles import CFG, limit
from .model import Command, body_vector, bounded, unit, wrap
from .three_recipient import move


CARRY_MOTOR = dict(drive=0.48, step_v=0.26, grip=12.0, pivot=3.4, cap=1.30)


def carrying(world, actor, desired, aim):
    error = wrap(aim - actor.yaw)
    turn = 0.0 if abs(error) < 0.035 else 4.8 * error
    return track_roller(world, actor, desired, turn, CARRY_MOTOR)


@dataclass(frozen=True)
class FiveReceive:
    cushion: float
    grip: float
    opening: float
    cushion_range: float = 0.58
    cushion_speed: float = 1.30

    def __post_init__(self):
        if not (
            0 <= self.cushion <= 1
            and 0 < self.grip <= 20
            and 0 <= self.opening <= 0.9
            and 0.25 <= self.cushion_range <= 1.6
            and 0 < self.cushion_speed <= CFG.max_speed
        ):
            raise ValueError("Invalid five-player reception profile")

    def command(self, world, actor, target):
        ball = world.ball
        bx, by = ball.x - actor.x, ball.y - actor.y
        gap = math.hypot(bx, by)
        speed = math.hypot(ball.vx, ball.vy)
        ux, uy = unit(ball.vx, ball.vy)
        along = (target[0] - ball.x) * ux + (target[1] - ball.y) * uy
        lead = max(0.0, min(along, speed**2 / (2 * ROLL_DECEL)))
        impact = ball.x + lead * ux, ball.y + lead * uy
        # Face the rolling lane until the ball is on the roller. A sideways or
        # cutback pass can leave a cone of headings open toward the goal.
        approach = math.atan2(-uy, -ux) if speed > 0.22 else math.atan2(by, bx)
        sign = world.direction(actor.key.team)
        gx, gy, clearance = shot_window(
            *impact, sign, [r for r in world.robots if r.key != actor.key]
        )
        aim = math.atan2(gy - impact[1], gx - impact[0])
        shootable = 0 < sign * (gx - impact[0]) < 2.45 and clearance > 0.13
        yaw = approach + limit(wrap(aim - approach), self.opening) if shootable else approach

        if gap > self.cushion_range:
            x = limit(impact[0] - CFG.possession_x * math.cos(yaw), 4.32)
            y = limit(impact[1] - CFG.possession_x * math.sin(yaw), 2.76)
            return move(actor, x, y, yaw, 1.55)

        # Aim the mouth at the approaching ball rather than driving the chassis
        # into it. Move with the pass at impact and damp error at the roller.
        offset_x, offset_y = (
            CFG.possession_x * math.cos(actor.yaw),
            CFG.possession_x * math.sin(actor.yaw),
        )
        forward, lateral = body_vector(bx, by, actor.yaw)
        centered = abs(forward - CFG.possession_x) < 0.028 and abs(lateral) < 0.032
        if centered and shootable:
            rel_speed = math.hypot(ball.vx - actor.vx, ball.vy - actor.vy)
            yaw += limit(wrap(aim - yaw), max(0.0, 0.75 - rel_speed))
        turn = limit(5 * wrap(yaw - actor.yaw), 3.2)
        dx, dy = self.grip * (bx - offset_x), self.grip * (by - offset_y)
        if not centered and speed > 0.45 and bx * ux + by * uy < 0:
            # Reposition across the lane, but do not let fore-aft correction
            # cancel the cushion while the ball is still closing on the mouth.
            along_lane = dx * ux + dy * uy
            correction = max(0.0, -0.20 - along_lane)
            dx, dy = dx + correction * ux, dy + correction * uy
        vx = self.cushion * ball.vx + dx + turn * offset_y
        vy = self.cushion * ball.vy + dy - turn * offset_x
        vx, vy = body_vector(vx, vy, actor.yaw)
        scale = min(1.0, self.cushion_speed / max(0.001, math.hypot(vx, vy)))
        return bounded(Command(scale * vx, scale * vy, turn, 1.0), CFG)
