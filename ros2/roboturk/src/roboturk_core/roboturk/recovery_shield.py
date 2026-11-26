"""Pressure-aware pivot and shot placement for a team that just won the ball."""

from dataclasses import replace
import math

import numpy as np

from .model import Command, RobotKey, body_vector, wrap
from .recovery_attack import AttackGame, center_dribbler
from .three_recipient import distance


def shot_target(world, actor):
    ball = world.ball
    gx = 4.53 * world.direction(actor.key.team)
    best = (-math.inf, 0.0)
    for gy in (-0.34, -0.23, -0.10, 0, 0.10, 0.23, 0.34):
        dx, dy = gx - ball.x, gy - ball.y
        length = max(0.01, math.hypot(dx, dy))
        ux, uy = dx / length, dy / length
        gap = 0.6
        for r in world.robots:
            if r.key == actor.key:
                continue
            rx, ry = r.x - ball.x, r.y - ball.y
            along = rx * ux + ry * uy
            if not 0 < along < length + 0.12:
                continue
            # Interception at ball arrival, with a limited velocity forecast.
            tau = min(0.36, along / 4.0)
            offset = rx * uy - ry * ux
            expected = offset + tau * (r.vx * uy - r.vy * ux)
            clearance = min(abs(expected), abs(offset) + 0.12)
            gap = min(gap, clearance)
        angle = math.atan2(dy, dx)
        quality = gap - 0.11 * abs(gy) - 0.018 * abs(wrap(angle - actor.yaw))
        best = max(best, (quality, gy))
    return gx, best[1]


class ShieldGame(AttackGame):
    def __init__(self, *args, controls, **kwargs):
        self.controls = controls
        super().__init__(*args, **kwargs)

    def commands(self):
        out = super().commands()
        if not self.option or not self.active or self.flight is not None:
            return out
        option = self.option
        team = option["team"]
        control = self.controls.get(team)
        if not control:
            return out
        actor = self.world.robot(RobotKey(team, option["actor"]))
        key = actor.key
        age = self.tick - option["tick"]
        action = option["action"]
        cmd = out[key]
        if action in ("pass", "shoot"):
            if action == "shoot":
                x, y = shot_target(self.world, actor)
                kick = 4.0
                angle = math.atan2(y - self.world.ball.y, x - self.world.ball.x)
            else:
                mate = self.world.robot(RobotKey(team, option["receiver"]))
                x, y = option.get("target", (mate.x, mate.y))
                kick = 2.2
                angle = math.atan2(y - actor.y, x - actor.x)
            err = wrap(angle - actor.yaw)
            cap = control["turn"]
            omega = float(np.clip(2 * cap * err, -cap, cap))
            cmd = center_dribbler(
                self.world, actor, Command(0, 0, omega, 1, kick if abs(err) < 0.024 else 0), age
            )
        rival = min(
            (r for r in self.world.robots if r.key.team != team), key=lambda r: distance(r, actor)
        )
        dx, dy = actor.x - rival.x, actor.y - rival.y
        dist = max(0.01, math.hypot(dx, dy))
        strength = control["escape"] * max(0.0, 1 - dist / 0.72)
        # Smooth translation away from the nearest challenger while pivoting
        # about the ball. Keep the normal continuous team's field limits.
        wx = float(np.clip(strength * dx / dist, -4.15 - actor.x, 4.15 - actor.x))
        wy = float(np.clip(strength * dy / dist, -2.65 - actor.y, 2.65 - actor.y))
        vx, vy = body_vector(wx, wy, actor.yaw)
        out[key] = replace(cmd, vx=cmd.vx + vx, vy=cmd.vy + vy)
        return out
