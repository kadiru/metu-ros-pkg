"""Close down a lead pass and protect the first touch before finishing."""

import math

from .lead_outlet import LeadGame
from .model import RobotKey, possession, wrap
from .three_match import CFG, face
from .three_recipient import distance, move
from .transition_attack import trap, unit


class PressureGame(LeadGame):
    def __init__(self, *args, press=True, **kwargs):
        self.press = press
        self.delivery = None
        self.shield_commands = []
        self.marker = None
        self.touch_tick = None
        self.screen_ticks = 0
        super().__init__(*args, **kwargs)

    def reset(self, team, reason):
        self.delivery = self.marker = self.touch_tick = None
        super().reset(team, reason)

    def commands(self):
        out = super().commands()
        w, b = self.world, self.world.ball
        if self.flight and self.flight is not self.delivery:
            self.delivery = self.flight
            key = RobotKey(self.flight["team"], self.flight["receiver"])
            self.runner = key
            self.touch_tick = None
            self.screen_ticks = 0
            # Choose the closest rival to the extrapolated rolling ball at release.
            rivals = [r for r in w.robots if r.key.team != key.team]
            self.marker = min(
                rivals, key=lambda r: math.hypot(r.x - b.x - 0.7 * b.vx, r.y - b.y - 0.7 * b.vy)
            ).key
        if not self.delivery:
            return out
        runner = w.robot(self.runner)
        held = possession(w, runner.key, CFG)
        if not self.flight and (not held or self.delivery.get("result") != "complete"):
            self.delivery = None
            return out
        marker = w.robot(self.marker)
        if self.press:
            t = min(0.65, distance(b, marker) / 1.6) if not held else 0.09
            out[marker.key] = move(
                marker,
                b.x + t * b.vx,
                b.y + t * b.vy,
                face(marker, b),
                speed=self.teams[marker.key.team].get("press_speed", 1.05),
            )
        p = self.teams[runner.key.team]
        if not held or "shield_speed" not in p:
            return out
        if self.touch_tick is None:
            self.touch_tick = self.tick
        rival = min(
            (r for r in w.robots if r.key.team != runner.key.team),
            key=lambda r: distance(r, runner),
        )
        ux, uy = unit(runner.x - rival.x, runner.y - rival.y)
        angle = math.atan2(uy, ux)
        if abs(wrap(angle - runner.yaw)) < 0.95:
            self.screen_ticks += 1
        else:
            self.screen_ticks = 0
        if getattr(self, "finished_delivery", None) is self.delivery:
            return out
        elapsed = self.tick - self.touch_tick
        if (self.screen_ticks >= p["dwell"] and elapsed >= p["hold"]) or elapsed >= 42:
            self.finished_delivery = self.delivery
            return out
        # Carry out of the challenge while the roller pivots around the captured ball.
        speed = p["shield_speed"]
        v = (speed * ux, speed * uy)
        out[runner.key] = trap(w, runner, angle, v, p | {"turn": p["shield_turn"]})
        if self.shield_commands and self.shield_commands[-1]["kick"] == self.delivery["tick"]:
            self.shield_commands[-1]["last"] = self.tick
        else:
            self.shield_commands.append(
                dict(
                    kick=self.delivery["tick"],
                    team=runner.key.team,
                    receiver=runner.key.id,
                    first=self.tick,
                    last=self.tick,
                )
            )
        self.choice = None
        return out
