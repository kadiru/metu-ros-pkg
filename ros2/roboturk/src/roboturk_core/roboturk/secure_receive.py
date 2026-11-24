"""Receive a rolling pass ahead of a closing marker, then settle it at the roller."""

from dataclasses import dataclass
import math

from .model import Command, body_vector, bounded, skill_limits, unit, wrap
from .team import TeamCheckpoint


@dataclass(frozen=True)
class ReceiveProfile:
    approach_speed: float
    cushion: float
    settle_gain: float

    def __post_init__(self):
        if not (
            0 < self.approach_speed <= 2 and 0 <= self.cushion <= 1 and 0 < self.settle_gain <= 25
        ):
            raise ValueError("Invalid reception motion parameters")

    def command(self, world, passer, receiver, cfg):
        p, r, ball = world.robot(passer), world.robot(receiver), world.ball
        # The passer holds its release pose. Approach from downfield and leave
        # sufficient space for a full-length transfer even on short passes.
        fx, fy = math.cos(p.yaw), math.sin(p.yaw)
        dx, dy = r.x - p.x, r.y - p.y
        downfield = dx * fx + dy * fy
        lateral = -dx * fy + dy * fx
        bx, by = ball.x - r.x, ball.y - r.y
        gap = math.hypot(bx, by)
        mouth = cfg.possession_x
        angle = math.atan2(by, bx)
        turn = 5.0 * wrap(angle - r.yaw)
        if gap > 0.48:
            inward = min(self.approach_speed, max(0.0, 4.5 * (downfield - 1.30)))
            side = max(-0.8, min(0.8, -4.5 * lateral))
            vx, vy = -inward * fx - side * fy, -inward * fy + side * fx
        else:
            # Bring the mouth onto the ball while yielding to its velocity.
            # A jaw already around the ball needs a small correction only.
            ux, uy = unit(bx, by)
            gain = self.settle_gain
            vx = gain * (bx - mouth * ux) + self.cushion * ball.vx
            vy = gain * (by - mouth * uy) + self.cushion * ball.vy
            axial = vx * fx + vy * fy
            limit = max(0.0, 4.5 * (downfield - 1.30))
            correction = max(0.0, -limit - axial)
            vx, vy = vx + correction * fx, vy + correction * fy
        vx, vy = body_vector(vx, vy, r.yaw)
        return bounded(Command(vx, vy, turn, 1.0), skill_limits(cfg))


class SecureTransfer(TeamCheckpoint):
    def __init__(self, checkpoint, cfg, profile):
        super().__init__(checkpoint, cfg)
        self.profile = profile

    def decode_action(self, action, world, passer, receiver, state, cfg):
        commands = super().decode_action(action, world, passer, receiver, state, cfg)
        if state.passed and not state.received:
            commands[receiver] = self.profile.command(world, passer, receiver, cfg)
        return commands
