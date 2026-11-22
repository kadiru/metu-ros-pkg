"""Control the roller during the receiver's transition from shielding to shooting."""

import math
from .early_outlet import EarlyOutletGame
from .model import possession
from .roller_rescue import rescue_motion
from .three_match import CFG


class CleanRecoveryGame(EarlyOutletGame):
    def commands(self):
        out = super().commands()
        if not self.delivery or getattr(self, "finished_delivery", None) is not self.delivery:
            return out
        w = self.world
        actor = w.robot(self.runner)
        p = self.teams[actor.key.team]
        if "finish_roller" not in p or not possession(w, actor.key, CFG):
            return out
        default = out[actor.key]
        if default.kick:
            return out
        rival = min(
            (r for r in w.robots if r.key.team != actor.key.team),
            key=lambda r: math.hypot(r.x - w.ball.x, r.y - w.ball.y),
        )
        # Preserve the shot decision and the kick. Limit a loss of grip on the preceding pivot.
        result = rescue_motion(w, actor, rival, default, p["finish_roller"], 100)
        if result is not None:
            out[actor.key] = result[0]
        return out
