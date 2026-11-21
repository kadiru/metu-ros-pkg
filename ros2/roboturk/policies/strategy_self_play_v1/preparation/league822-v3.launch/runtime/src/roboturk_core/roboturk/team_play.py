"""Deployable two-robot play: learned transfer, then learned scoring.

The caller supplies measured substep worlds and applied-kick feedback. This core
API is available to a Supervisor integration; ROS does not yet carry kick feedback.
"""

import math
from .model import Command, Config, Intent, Referee, Skill, possession
from .team import TransferState


class PassReceiveScorePlay:
    def __init__(self, transfer_policy, shooting_policy, cfg=Config()):
        self.transfer_policy, self.shooting_policy, self.cfg = transfer_policy, shooting_policy, cfg
        self.state = TransferState()
        self.passer = self.receiver = None
        self.last_time = None

    def reset(self, world, passer, receiver):
        if passer == receiver or passer.team != receiver.team:
            raise ValueError("Pass roles require distinct teammates")
        world.robot(passer)
        world.robot(receiver)
        self.passer, self.receiver = passer, receiver
        self.state = (
            self.transfer_policy.initial_state(world, passer, receiver, self.cfg)
            if self.transfer_policy is not None
            else TransferState()
        )
        self.last_time = world.time
        if hasattr(self.shooting_policy, "reset"):
            self.shooting_policy.reset()

    def commands(self, world):
        if self.passer is None:
            raise RuntimeError("Assign roles with reset before running the play")
        if (
            world.referee != Referee.RUNNING
            or not world.ball.visible
            or not all(world.robot(k).visible for k in (self.passer, self.receiver))
        ):
            return {self.passer: Command(), self.receiver: Command()}
        if self.state.received:
            goal = (world.direction(self.receiver.team) * self.cfg.length / 2, 0.0)
            return {
                self.passer: Command(),
                self.receiver: self.shooting_policy.command(
                    world, self.receiver, Intent(Skill.SHOOT, goal), self.cfg
                ),
            }
        action = self.transfer_policy.predict(
            world, self.passer, self.receiver, self.state, self.cfg
        )
        return self.transfer_policy.decode_action(
            action, world, self.passer, self.receiver, self.state, self.cfg
        )

    def observe(self, samples, kick_events):
        if not samples:
            raise ValueError("Measured physics samples are required")
        current = samples[-1]
        if self.last_time is None or current.time <= self.last_time:
            raise ValueError("Reset on episode/time rewind; do not count duplicate samples")
        if not math.isclose(current.time - self.last_time, self.cfg.dt, abs_tol=1e-7):
            raise ValueError("Play requires one observation per control period")
        self.last_time = current.time
        s = self.state
        if any(key == self.passer for key, _, _ in kick_events):
            s.passed = True
        if not s.passed:
            r, b = current.robot(self.passer), current.ball
            stable = (
                all(possession(w, self.passer, self.cfg) for w in samples)
                and math.hypot(r.vx - b.vx, r.vy - b.vy) < 0.2
            )
            s.passer_dwell = s.passer_dwell + 1 if stable else 0
        elif not s.received:
            r, b = current.robot(self.receiver), current.ball
            stable = (
                all(possession(w, self.receiver, self.cfg) for w in samples)
                and math.hypot(r.vx - b.vx, r.vy - b.vy) < 0.2
            )
            s.receiver_dwell = s.receiver_dwell + 1 if stable else 0
            s.received = s.receiver_dwell >= 20
