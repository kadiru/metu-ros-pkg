"""Turn around the ball under roller control before a flat pass or shot."""

from dataclasses import dataclass
import math

from . import field
from .carrier_motor import track_roller
from .model import RobotKey, body_vector, wrap

TURN_MOTOR = dict(drive=0.65, step_v=0.27, grip=15.0, pivot=4.5, cap=1.15)
MAX_DRIBBLE = 0.85


def turn_rate(cfg):
    return min(
        TURN_MOTOR["pivot"],
        cfg.max_omega,
        cfg.max_speed / (cfg.possession_x + 0.045),
        math.sqrt(0.6 * cfg.max_acceleration / cfg.possession_x),
    )


def arc_clear(world, actor, sweep, rate, cfg, observed, clearance):
    """Check the dribbler and chassis along the proposed sweep using observed motion."""
    ball = world.ball
    radius = cfg.possession_x
    if abs(ball.x) > cfg.length / 2 - 0.30 or abs(ball.y) > cfg.width / 2 - 0.30:
        return False
    samples = max(1, math.ceil(abs(sweep) / 0.14))
    neighbors = [r for r in world.robots if r.key != actor.key and r.visible]
    for i in range(samples + 1):
        angle = actor.yaw + sweep * i / samples
        c, s = math.cos(angle), math.sin(angle)
        x, y = ball.x - radius * c, ball.y - radius * s
        if abs(x) > cfg.length / 2 - 0.15 or abs(y) > cfg.width / 2 - 0.15:
            return False
        padding = cfg.robot_radius + 0.015
        if (
            abs(x) > cfg.length / 2 - field.DEFENSE_DEPTH - padding
            and abs(y) < field.DEFENSE_WIDTH / 2 + padding
        ):
            return False
        ahead = min(0.48, abs(sweep) * i / (samples * rate))
        for other in neighbors:
            vx, vy = observed.get(other.key, (0.0, 0.0))
            qx, qy = other.x + ahead * vx, other.y + ahead * vy
            if math.hypot(x - qx, y - qy) < clearance:
                return False
            dx, dy = qx - ball.x, qy - ball.y
            gap = math.hypot(dx, dy)
            if other.key.team != actor.key.team and gap < 0.27 and dx * c + dy * s > 0.5 * gap:
                return False
    return True


@dataclass
class Turn:
    actor: RobotKey
    receiver: RobotKey | None
    target: tuple[float, float]
    origin: tuple[float, float]
    started: float
    sweep: float
    reverse: bool
    last_yaw: float
    turned: float = 0.0

    def describe(self):
        return dict(
            direction="left" if self.sweep > 0 else "right",
            reverse=self.reverse,
            degrees=round(abs(math.degrees(self.turned)), 1),
        )


class SpinRelease:
    def __init__(self):
        self.owner = None
        self.grip_origin = None
        self.attempted = False
        self.turn = None

    def observe(self, world, holders, sole):
        if self.owner not in holders or (sole and sole != self.owner):
            self.owner = sole
            self.grip_origin = (world.ball.x, world.ball.y)
            self.attempted = False
        if self.turn and self.turn.actor != sole:
            self.turn = None

    def start(self, world, actor, target, receiver, cfg, observed, clearance):
        if self.attempted or self.owner != actor.key or actor.key.id == 0:
            return None
        ball = world.ball
        origin = (ball.x, ball.y)
        if (
            self.grip_origin is None
            or math.dist(origin, self.grip_origin) > MAX_DRIBBLE - 0.15
            or math.hypot(ball.vx, ball.vy) > 0.53
            or math.hypot(actor.vx, actor.vy) > 0.65
        ):
            return None
        yaw = math.atan2(target[1] - ball.y, target[0] - ball.x)
        short = wrap(yaw - actor.yaw)
        rate = turn_rate(cfg)
        if abs(short) < 0.95 or rate < 1.8:
            return None
        for sweep in (short, short - math.copysign(math.tau, short)):
            if abs(sweep) / rate > 1.55:
                continue
            if arc_clear(world, actor, sweep, rate, cfg, observed, clearance):
                self.turn = Turn(
                    actor.key,
                    receiver,
                    target,
                    origin,
                    world.time,
                    sweep,
                    sweep != short,
                    actor.yaw,
                )
                self.attempted = True
                return self.turn
        return None

    def step(self, world, actor, target, cfg, observed, clearance):
        """Return the coupled drive and yaw command, plus a release-ready flag."""
        turn = self.turn
        ball = world.ball
        position = (ball.x, ball.y)
        rate = turn_rate(cfg)
        if (
            rate < 1.8
            or world.time - turn.started > 1.25 + abs(turn.sweep) / rate
            or math.dist(position, self.grip_origin) > MAX_DRIBBLE
            or math.dist(position, turn.origin) > 0.23
            or math.dist(target, turn.target) > 0.25
        ):
            self.turn = None
            return None
        turn.turned += wrap(actor.yaw - turn.last_yaw)
        turn.last_yaw = actor.yaw
        aim = math.atan2(target[1] - ball.y, target[0] - ball.x)
        error = wrap(aim - actor.yaw)
        remaining = turn.sweep - turn.turned
        # Unwrap about the committed sweep, keeping the chosen direction across ±π.
        error += math.tau * round((remaining - error) / math.tau)
        if not arc_clear(world, actor, error, rate, cfg, observed, clearance):
            self.turn = None
            return None
        forward, lateral = body_vector(ball.x - actor.x, ball.y - actor.y, actor.yaw)
        ready = (
            abs(error) < 0.043
            and abs(actor.omega) < 0.36
            and abs(forward - cfg.possession_x) < 0.022
            and abs(lateral) < 0.026
        )
        omega = max(-rate, min(rate, 5.8 * error))
        step = 12.0 * cfg.dt
        omega = (
            0.0
            if actor.omega * omega < 0
            else max(actor.omega - step, min(actor.omega + step, omega))
        )
        ax, ay = turn.origin[0] - ball.x, turn.origin[1] - ball.y
        drive = min(3.0, 0.30 / max(0.001, math.hypot(ax, ay)))
        return track_roller(world, actor, (drive * ax, drive * ay), omega, TURN_MOTOR), ready
