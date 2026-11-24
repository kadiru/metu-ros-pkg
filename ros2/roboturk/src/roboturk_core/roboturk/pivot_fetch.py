"""Bounded roller sweeps to free a ball after a stalled Presser challenge."""

from dataclasses import dataclass
import math

from .five_roles import CFG
from .model import Command, RobotKey, body_vector, possession, world_vector, wrap
from .three_recipient import move


@dataclass
class Duel:
    carrier: RobotKey
    last_time: float
    shared_ticks: int = 0
    attempts: int = 0
    started: float | None = None
    last_sweep: float | None = None
    yaw_target: float = 0
    side: float = 1


class PivotFetch:
    def __init__(self, tackle):
        self.tackle = tackle
        self.duels = {}
        self.sweeps = []

    def command(self, world, presser, carrier, observed, has_ball):
        key = presser.key
        now = world.time
        duel = self.duels.get(key)
        if (
            duel is None
            or duel.carrier != carrier.key
            or not 0 < now - duel.last_time <= 1.5 * CFG.dt
        ):
            duel = self.duels[key] = Duel(carrier.key, now)
        duel.last_time = now
        if not has_ball or not possession(world, carrier.key, CFG):
            duel.shared_ticks = 0
            duel.started = None
            return self.tackle.command(world, presser, carrier, observed, has_ball)

        duel.shared_ticks += 1
        forward = (math.cos(carrier.yaw), math.sin(carrier.yaw))
        nx, ny = forward
        if duel.started is None and duel.shared_ticks >= 8 and duel.attempts < 2:
            cross = nx * (presser.y - carrier.y) - ny * (presser.x - carrier.x)
            duel.side = math.copysign(1.0, cross)
            direction = -duel.side if duel.attempts == 0 else duel.side
            duel.yaw_target = presser.yaw + direction * math.radians(48)
            duel.started = now
            duel.attempts += 1
            self.sweeps.append((now, key, carrier.key))

        if duel.started is None:
            return self.tackle.command(world, presser, carrier, observed, has_ball)
        if now - duel.started >= 0.68:
            duel.started = None
            duel.shared_ticks = 0
            return self.tackle.command(world, presser, carrier, observed, has_ball)

        duel.last_sweep = now
        # Pivot the body while translating its center so that the existing
        # roller spring sweeps sideways across the carrier's mouth.
        omega = max(-3.2, min(3.2, 9.0 * wrap(duel.yaw_target - presser.yaw)))
        ball = world.ball
        lateral = 0.057 * duel.side
        mx, my = body_vector(0.024 * nx - lateral * ny, 0.024 * ny + lateral * nx, presser.yaw)
        mx = max(-0.032, min(0.032, mx))
        my = max(-0.041, min(0.041, my))
        bx, by = body_vector(ball.x - presser.x, ball.y - presser.y, presser.yaw)
        ex, ey = world_vector(bx - CFG.possession_x + mx, by + my, presser.yaw)
        gain = min(8.0, 0.31 / max(0.001, math.hypot(ex, ey)))
        ox = CFG.possession_x * math.cos(presser.yaw)
        oy = CFG.possession_x * math.sin(presser.yaw)
        x, y = body_vector(
            ball.vx + gain * ex + omega * oy, ball.vy + gain * ey - omega * ox, presser.yaw
        )
        return Command(x, y, omega, 1)

    def collect_loose(self, world, presser):
        duel = self.duels.get(presser.key)
        if (
            duel is None
            or duel.last_sweep is None
            or not 0 < world.time - duel.last_sweep <= 0.28
            or any(possession(world, r.key, CFG) for r in world.robots)
        ):
            return None
        ball = world.ball
        x, y = ball.x + 0.10 * ball.vx, ball.y + 0.10 * ball.vy
        dx, dy = x - presser.x, y - presser.y
        if math.hypot(dx, dy) > 0.31:
            return None
        angle = math.atan2(dy, dx)
        return move(
            presser,
            x - CFG.possession_x * math.cos(angle),
            y - CFG.possession_x * math.sin(angle),
            angle,
            speed=0.65,
        )
