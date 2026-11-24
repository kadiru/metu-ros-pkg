"""Challenge the frozen pass-receive-score play with a single closing marker."""

from dataclasses import asdict, replace
import math

import numpy as np

from .backend import ReferenceBackend
from .carrier_motor import MOTOR, track_roller
from .defended import DEFENDER, DefendedBackend, OpposedPassEnv
from .model import Command, Config, Robot, possession, unit, wrap
from .team import TeamCheckpoint
from .three_recipient import move


class ClosingDefender:
    def __init__(self, speed):
        self.speed = speed
        self.press_start = None

    def command(self, world):
        marker, ball = world.robot(DEFENDER), world.ball
        angle = math.atan2(ball.y - marker.y, ball.x - marker.x)
        if self.press_start is None:
            if math.hypot(ball.vx, ball.vy) < 0.9:
                return Command(omega=max(-5, min(5, 5 * wrap(angle - marker.yaw))))
            self.press_start = world.time
        # The 3v3 closing marker also pursues a short extrapolation of observed ball motion.
        travel = math.hypot(ball.x - marker.x, ball.y - marker.y)
        lead = min(0.65, travel / 1.6)
        return move(marker, ball.x + lead * ball.vx, ball.y + lead * ball.vy, angle, self.speed)


class ClosingBackend(DefendedBackend):
    def reset(self, world):
        receiver = world.robots[1]
        side = int(self.rng.choice([-1, 1]))
        distance = self.rng.uniform(1.2, 2.25)
        if abs(receiver.y + side * distance) > self.cfg.width / 2 - 0.25:
            side = -side
        x, y = receiver.x + self.rng.uniform(-0.4, 0.4), receiver.y + side * distance
        marker = Robot(DEFENDER, x, y, math.atan2(world.ball.y - y, world.ball.x - x))
        self.defender = ClosingDefender(float(self.rng.uniform(0.45, 1.05)))
        self.receiver = receiver.key
        self.first_touch = None
        world = replace(world, robots=(*world.robots, marker))
        self.opening = asdict(world)
        return super().reset(world)

    def step(self, commands):
        world = super().step(commands)
        if self.defender.press_start is not None and self.first_touch is None:
            for sample in getattr(self.base, "last_substep_worlds", (world,)):
                if possession(sample, self.receiver, self.cfg):
                    marker, receiver = sample.robot(DEFENDER), sample.robot(self.receiver)
                    self.first_touch = {
                        "seconds": sample.time,
                        "marker_distance": math.hypot(marker.x - receiver.x, marker.y - receiver.y),
                    }
                    break
        return world

    def evidence(self):
        marker, receiver = self.world.robot(DEFENDER), self.world.robot(self.receiver)
        return {
            **super().evidence(),
            "opponent": "closing_marker_v1",
            "opening": self.opening,
            "press_start_seconds": self.defender.press_start,
            "first_receiver_touch": self.first_touch,
            "marker_receiver_distance": math.hypot(marker.x - receiver.x, marker.y - receiver.y),
        }


class ClosingPassEnv(OpposedPassEnv):
    task_version = "pass_receive.closing_marker.v1"

    def __init__(self, cfg=Config(), backend=None):
        opposed = ClosingBackend(backend or ReferenceBackend(cfg), cfg)
        opposed.name = opposed.base.name + "+closing_marker_v1"
        super().__init__(cfg, opposed)

    def reset(self, *, seed=None, options=None):
        # Independent stream keeps the published two-teammate opening distribution.
        self.backend.rng = np.random.default_rng(np.random.SeedSequence(seed, spawn_key=(1,)))
        return super().reset(seed=seed, options=options)


class ClosingCarry:
    """Carry while turning, then hand the finish back to the published scoring policy."""

    def __init__(self, scorer, speed, avoidance):
        if not (0 < speed <= 2 and 0 <= avoidance <= 2):
            raise ValueError("Invalid carry speed or avoidance weight")
        self.scorer, self.speed, self.avoidance = scorer, speed, avoidance

    def command(self, world, key, intent, cfg):
        return self.motion(world, key, intent.target, cfg) or self.scorer.command(
            world, key, intent, cfg
        )

    def motion(self, world, key, target, cfg):
        b, actor, marker = world.ball, world.robot(key), world.robot(DEFENDER)
        dx, dy = target[0] - b.x, target[1] - b.y
        distance = math.hypot(dx, dy)
        if distance <= 1.55 or not possession(world, key, cfg):
            return None
        gx, gy = unit(dx, dy)
        mx, my = b.x - marker.x, b.y - marker.y
        gap = math.hypot(mx, my)
        weight = self.avoidance * max(0, 1 - gap / 1.3)
        ax, ay = unit(mx, my)
        vx, vy = unit(gx + weight * ax, gy + weight * ay)
        speed = min(self.speed, 0.45 + 1.4 * (distance - 1.55))
        omega = max(-2.3, min(2.3, 3.2 * wrap(math.atan2(dy, dx) - actor.yaw)))
        return track_roller(world, actor, (speed * vx, speed * vy), omega, MOTOR)


class FirstTouchTransfer(TeamCheckpoint):
    """Start carrying during the measured hold. Only the scorer can request a shot."""

    def __init__(self, checkpoint, cfg, carry, first_touch_steps):
        super().__init__(checkpoint, cfg)
        if not isinstance(first_touch_steps, int) or not 1 <= first_touch_steps < 20:
            raise ValueError("First-touch delay must be 1 to 19 control steps")
        self.carry, self.first_touch_steps = carry, first_touch_steps

    def decode_action(self, action, world, passer, receiver, state, cfg):
        commands = super().decode_action(action, world, passer, receiver, state, cfg)
        if state.passed and state.receiver_dwell >= self.first_touch_steps:
            goal = (world.direction(receiver.team) * cfg.length / 2, 0.0)
            motion = self.carry.motion(world, receiver, goal, cfg)
            if motion is not None:
                commands[receiver] = motion
        return commands
