"""Prepare the running outlet while the carrier is still establishing its first touch."""

import math
from dataclasses import replace

from .blocked_outlet import geometry
from .lead_outlet import margin, rendezvous, run_direction
from .model import possession, wrap
from .three_match import CFG
from .three_recipient import candidates, move
from .touch_escape import EscapeGame


def early_route(w, actor, p):
    b = w.ball
    enemies = [r for r in w.robots if r.key.team != actor.key.team]
    best = None
    for runner in candidates(w, actor):
        direction = run_direction(w, runner, p)
        target, center = rendezvous(w, runner, direction, p)
        angle = math.atan2(target[1] - b.y, target[0] - b.x)
        _, lane, space = geometry(
            (b.x, b.y),
            w.direction(actor.key.team),
            (runner.x, runner.y),
            [(r.x, r.y) for r in enemies],
        )
        clearance = margin(b, target, enemies, p)
        score = (
            2.8 * min(0.75, clearance)
            + 0.65 * min(1.3, space)
            - 3.5 * max(0, 0.30 - lane)
            - 2 * max(0, 0.60 - space)
            - 0.22 * abs(math.hypot(target[0] - b.x, target[1] - b.y) - 1.1)
            - p["early_angle_cost"] * abs(wrap(angle - actor.yaw))
        )
        if best is None or score > best[0]:
            best = score, runner, center, angle, clearance
    return best[1:]


class EarlyOutletGame(EscapeGame):
    def __init__(self, *args, **kwargs):
        self.early_plans = []
        super().__init__(*args, **kwargs)

    def commands(self):
        out = super().commands()
        if self.active or self.flight or not self.contestants:
            return out
        w = self.world
        held = [r for r in w.robots if possession(w, r.key, CFG)]
        if len(held) != 1 or held[0].key not in self.contestants:
            return out
        actor = held[0]
        p = self.teams[actor.key.team]
        if "early_angle_cost" not in p or self.sole_count < p["early_samples"]:
            return out
        runner, center, angle, gap = early_route(w, actor, p)
        if gap < p["early_lane"]:
            return out
        out[runner.key] = move(runner, *center, angle + math.pi, speed=p["run_speed"])
        key = (actor.key.team, actor.key.id, runner.key.id)
        prev = self.early_plans[-1] if self.early_plans else {}
        if prev.get("players") != key or prev.get("last") != self.tick - 1:
            self.early_plans.append(dict(players=key, first=self.tick, clearance=gap))
        self.early_plans[-1].update(last=self.tick, target_angle=angle)
        default = out[actor.key]
        error = wrap(angle - actor.yaw)
        if abs(error) > p["early_cone"]:
            return out
        # Pivot about the moving physical contact while keeping the inherited escape velocity.
        omega = max(-p["early_pivot"], min(p["early_pivot"], 6.5 * error))
        extra = p["early_blend"] * (omega - default.omega)
        c, s = math.cos(actor.yaw), math.sin(actor.yaw)
        dx, dy = w.ball.x - actor.x, w.ball.y - actor.y
        x, y = c * dx + s * dy, c * dy - s * dx
        out[actor.key] = replace(
            default,
            omega=default.omega + extra,
            vx=default.vx + extra * y,
            vy=default.vy - extra * x,
            kick=0,
        )
        return out
