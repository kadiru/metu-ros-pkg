"""Learn where to intercept the next kick and stabilize the first receiving touch."""

import math
from .carrier_motor import track_roller
from .model import possession, wrap
from .takeaway_outlet import OutletGame
from .three_match import CFG, face
from .three_recipient import distance, move
from .transition_attack import unit, window


def interception(ball, players, profile):
    speed = math.hypot(ball.vx, ball.vy)
    ux, uy = unit(ball.vx, ball.vy)
    best = None
    for r in players:
        for i in range(1, 14):
            t = 0.065 * i
            travel = max(0, speed * t - 0.23 * t * t)
            x = ball.x + ux * (travel + 0.115)
            y = ball.y + uy * (travel + 0.115)
            if abs(x) > 4.32 or abs(y) > 2.83:
                continue
            miss = max(0, math.hypot(x - r.x, y - r.y) - 1.12 * t)
            score = (
                3.5 * miss
                + abs(t - profile["time"]) * 0.36
                + 0.07 * abs(wrap(math.atan2(-uy, -ux) - r.yaw))
            )
            if best is None or score < best[0]:
                best = (score, r, x, y, math.atan2(-uy, -ux))
    return best


class InterceptGame(OutletGame):
    def __init__(self, *args, press_from=0, **kwargs):
        self.press_from = press_from
        super().__init__(*args, **kwargs)

    def formation(self, out, held):
        super().formation(out, held)
        if self.tick * 4 < self.press_from:
            return
        w = self.world
        b = w.ball
        for team, p in self.teams.items():
            profile = p.get("takeaway_press")
            if not profile or any(k.team == team for k in held):
                continue
            if self.flight and self.flight["team"] == team:
                continue
            players = [r for r in w.robots if r.key.team == team and r.key not in self.contestants]
            if not players:
                continue
            if (
                not held
                and self.last_control_team not in (None, team)
                and math.hypot(b.vx, b.vy) > 0.40
            ):
                result = interception(b, players, profile)
                if result:
                    _, r, x, y, a = result
                    out[r.key] = move(r, x, y, a, speed=1.2)
                continue
            if len(held) != 1:
                continue
            carrier = w.robot(next(iter(held)))
            dx, dy = unit(-4.5 * w.direction(team) - b.x, -b.y)
            ax, ay = math.cos(carrier.yaw), math.sin(carrier.yaw)
            u = profile["face"]
            ux, uy = unit(u * ax + (1 - u) * dx, u * ay + (1 - u) * dy)
            x = b.x + profile["stand"] * ux
            y = b.y + profile["stand"] * uy
            first = min(players, key=lambda r: math.hypot(r.x - x, r.y - y))
            out[first.key] = move(
                first,
                max(-4.31, min(4.31, x)),
                max(-2.80, min(2.80, y)),
                face(first, b),
                speed=1.16,
            )
            if not profile["screen"] or len(players) < 2:
                continue
            mates = [r for r in w.robots if r.key.team == carrier.key.team and r.key != carrier.key]
            receiver = max(
                mates,
                key=lambda r: (
                    math.cos(face(carrier, r) - carrier.yaw) + 0.15 * distance(carrier, r)
                ),
            )
            x = b.x + 0.65 * (receiver.x - b.x)
            y = b.y + 0.65 * (receiver.y - b.y)
            screen = min(
                (r for r in players if r != first), key=lambda r: math.hypot(r.x - x, r.y - y)
            )
            out[screen.key] = move(
                screen,
                max(-4.28, min(4.28, x)),
                max(-2.78, min(2.78, y)),
                face(screen, b),
                speed=1.1,
            )

    def commands(self):
        out = super().commands()
        if self.tick * 4 < self.press_from:
            return out
        w = self.world
        b = w.ball
        k = self.control_key
        if k is None or self.last_control_team in (None, k.team) or self.control_age >= 20:
            return out
        p = self.teams[k.team].get("takeaway_press")
        if not p or not p["settle"] or not possession(w, k, CFG):
            return out
        actor = w.robot(k)
        rival = min((r for r in w.robots if r.key.team != k.team), key=lambda r: distance(r, b))
        ux, uy = unit(b.x - rival.x, b.y - rival.y)
        tx, ty = unit(4.5 * w.direction(k.team) - b.x, -b.y)
        vx = 0.68 * ux + 0.30 * tx
        vy = 0.68 * uy + 0.30 * ty
        if abs(b.y) > 2.5:
            vy -= 0.65 * math.copysign(1, b.y)
        gx, gy, _ = window(w, actor)
        angle = math.atan2(gy - b.y, gx - b.x)
        out[k] = track_roller(
            w,
            actor,
            (vx, vy),
            4.5 * wrap(angle - actor.yaw),
            dict(drive=0.42, step_v=0.30, grip=23, pivot=3.6, cap=1.15),
        )
        return out
