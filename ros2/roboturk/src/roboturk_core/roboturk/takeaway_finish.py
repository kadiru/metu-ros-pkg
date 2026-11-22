"""Calibrate shot release during the outside escape using only the current field."""

from dataclasses import replace
import math

from .model import possession, wrap
from .takeaway_press import InterceptGame
from .takeaway_sprint import sprint
from .three_match import CFG
from .three_recipient import distance


class FinishGame(InterceptGame):
    def counter_commands(self, out):
        c = self.counter
        if not c or c["action"] != 9:
            return super().counter_commands(out)
        team = c["team"]
        profile = self.teams[team].get("escape_finish")
        if not profile:
            return super().counter_commands(out)
        w = self.world
        held = [r for r in w.robots if possession(w, r.key, CFG)]
        if len(held) != 1 or held[0].key.team != team:
            return super().counter_commands(out)
        r = held[0]
        b = w.ball
        if "speed" in profile:
            if self.flight:
                return super().counter_commands(out)
            self.owner = r.key
            self.choice = None
            self.release_plans.pop(self.tick, None)
            out[r.key] = sprint(w, r, profile)
            return out
        gap = min(distance(b, q) for q in w.robots if q.key.team != team)
        if self.tick - c["sample"] // 4 >= profile["handoff_ticks"] and gap > 0.45:
            self.counter = None
            self.option = None
            return out
        out = super().counter_commands(out)
        cmd = out[r.key]
        if self.flight:
            return out
        # Give the rolling ball room to deflect slightly at release. Prefer a
        # central crossing and keep turning toward goal while waiting for it.
        gx = 4.53 * w.direction(team)
        ux = math.cos(r.yaw)
        if abs(ux) > 0.15 and (gx - b.x) / ux > 0:
            crossing = b.y + (gx - b.x) * math.tan(r.yaw)
            if abs(crossing) > profile["margin"] and cmd.kick:
                out[r.key] = replace(cmd, kick=0)
            if gap > 0.4 and abs(crossing) > profile["margin"]:
                aim = math.atan2(-b.y, gx - b.x)
                out[r.key] = replace(
                    out[r.key],
                    omega=max(-6.6, min(6.6, cmd.omega + profile["centering"] * wrap(aim - r.yaw))),
                )
        return out
