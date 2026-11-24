"""A closing field defender and a frozen neural goalkeeper against the secure receiver."""

from dataclasses import dataclass
import math

import numpy as np

from .carrier_motor import MOTOR, track_roller
from .closing_press import ClosingBackend, ClosingCarry
from .defended import OpposedPassEnv
from .keeper import KEEPER
from .learned_opponents import LearnedKeeperBackend
from .model import Command, possession, unit, wrap
from .shoot_contract import SHOT_SPEED


class GuardedPassEnv(OpposedPassEnv):
    task_version = "pass_receive.closing_marker_learned_keeper.v1"

    def __init__(self, cfg, backend, keeper):
        marker = ClosingBackend(backend, cfg)
        marker.name = backend.name + "+closing_marker_v1"
        super().__init__(cfg, LearnedKeeperBackend(marker, cfg, keeper))

    def reset(self, *, seed=None, options=None):
        marker_rng, keeper_rng = (
            np.random.default_rng(np.random.SeedSequence(seed, spawn_key=(stream,)))
            for stream in (1, 2)
        )
        self.backend.base.rng = marker_rng
        self.backend.keeper.configure(keeper_rng)
        return super().reset(seed=seed, options=options)


@dataclass(frozen=True)
class FinishProfile:
    release_range: float
    corner_offset: float
    prediction: float

    def __post_init__(self):
        if not (
            0.55 <= self.release_range <= 1.65
            and 0.2 <= self.corner_offset <= 0.46
            and 0 <= self.prediction <= 1
        ):
            raise ValueError("Invalid guarded shooting profile")


class GuardedFinish:
    """Carry into range, select the more open corner and release on the observed shot ray."""

    def __init__(self, carry: ClosingCarry, profile: FinishProfile):
        self.carry, self.profile = carry, profile
        self.reset()

    def reset(self):
        self.side = None
        self.fired = False

    def command(self, world, key, intent, cfg):
        ball, actor, keeper = world.ball, world.robot(key), world.robot(KEEPER)
        goal_x, goal_y = intent.target
        distance = math.hypot(goal_x - ball.x, goal_y - ball.y)
        if self.fired:
            return Command()
        if distance > 1.7 and self.side is None:
            motion = self.carry.motion(world, key, intent.target, cfg)
            if motion is not None:
                return motion
        if not possession(world, key, cfg):
            return self.carry.scorer.command(world, key, intent, cfg)

        sign = world.direction(key.team)
        depth = max(0.02, sign * (goal_x - ball.x))
        line_fraction = (
            max(0.0, min(1.0, (keeper.x - ball.x) / (goal_x - ball.x))) if depth > 0.02 else 0.0
        )
        # Project the observed keeper onto each possible shot line at its own depth.
        flight = distance * line_fraction / SHOT_SPEED
        keeper_y = keeper.y + self.profile.prediction * flight * keeper.vy
        corner = self.profile.corner_offset
        if self.side is None and distance <= self.profile.release_range + 0.16:

            def clearance(side):
                return abs(ball.y + line_fraction * (side * corner - ball.y) - keeper_y)

            self.side = max((-1, 1), key=clearance)
        target_y = self.side * corner if self.side is not None else 0.0
        heading = math.atan2(target_y - ball.y, goal_x - ball.x)
        error = wrap(heading - actor.yaw)
        if self.side is not None and distance <= self.profile.release_range and abs(error) < 0.027:
            self.fired = True
            return Command(kick=SHOT_SPEED)

        # Keep rolling forward during the turn to avoid the press. Brake short of the keeper.
        stop_x = goal_x - sign * 0.51
        dx, dy = stop_x - ball.x, 0.25 * target_y - ball.y
        ux, uy = unit(dx, dy)
        speed = min(1.25, 2.5 * math.hypot(dx, dy))
        omega = max(-3.8, min(3.8, 8 * error))
        return track_roller(world, actor, (speed * ux, speed * uy), omega, MOTOR)
