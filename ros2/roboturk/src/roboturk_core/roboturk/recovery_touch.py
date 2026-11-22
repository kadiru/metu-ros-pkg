"""First-touch choices for the original carrier of a contested recovery."""

import math

from .model import RobotKey, body_vector, possession, wrap
from .pressure_receive import PressureGame
from .three_match import CFG, face
from .three_recipient import distance, move
from .transition_attack import trap, unit


def touch_choice(w, actor, rival, intended, p):
    b = w.ball
    x, y = body_vector(b.x - actor.x, b.y - actor.y, actor.yaw)
    vx, vy = body_vector(b.vx - actor.vx, b.vy - actor.vy, actor.yaw)
    # Predict roller-relative drift, including the rotating chassis.
    dx = x - 0.115 + p["look"] * (vx + actor.omega * y)
    dy = y + p["look"] * (vy - actor.omega * x)
    edge = max(abs(dx) / 0.035, abs(dy) / 0.045)
    speed = math.hypot(b.vx, b.vy)
    close = max(0, 1 - distance(actor, rival) / 0.50)
    rush = max(0, speed - p["cruise"])
    danger = edge + p["rush_weight"] * rush
    turn = abs(wrap(face(b, intended) - actor.yaw)) / math.pi
    # Release includes preparing the synchronized pass when its kick gates are still closed.
    values = (
        p["release_value"],
        danger + p["shield_bias"] * close,
        danger + p["side_bias"] * turn + 0.32 * (1 - close) - p["side_cost"],
    )
    i = max(range(3), key=values.__getitem__)
    return ("release", "shield", "sidestep")[i], values


class RecoveryTouchGame(PressureGame):
    def __init__(self, *args, **kwargs):
        self.first_touches = []
        super().__init__(*args, **kwargs)

    def commands(self):
        out = super().commands()
        w, b = self.world, self.world.ball
        if self.flight or self.delivery:
            return out
        if self.active:
            team, start = self.active
            if any(f["team"] == team and f["tick"] * 4 >= start for f in self.passes):
                return out
        elif not self.contestants:
            return out
        held = [r for r in w.robots if possession(w, r.key, CFG)]
        if len(held) != 1:
            return out
        actor = held[0]
        team = actor.key.team
        p = self.teams[team]
        if "release_value" not in p or (not self.active and actor.key not in self.contestants):
            return out
        if self.active and (team != self.active[0] or self.tick * 4 - self.active[1] > p["window"]):
            return out
        rival = min((r for r in w.robots if r.key.team != team), key=lambda r: distance(b, r))
        option = self.option
        if option and option["actor"] == actor.key.id:
            mate = w.robot(RobotKey(team, option["receiver"]))
        else:
            mate = max(
                (r for r in w.robots if r.key.team == team and r.key != actor.key),
                key=lambda r: min(distance(r, z) for z in w.robots if z.key.team != team),
            )
        action, values = touch_choice(w, actor, rival, mate, p)
        if (
            p.get("offer")
            and self.active
            and option
            and option["action"] == "dribble"
            and action == "release"
        ):
            # There is no synchronized kick on this lane. Carry until the next pass decision.
            action = "sidestep"
        original = out[actor.key]
        if original.kick and self.choice and p.get("take_open", True):
            action = "release"
        if (
            not self.first_touches
            or self.first_touches[-1]["action"] != action
            or self.first_touches[-1]["last"] != self.tick - 1
            or self.first_touches[-1]["team"] != team
            or self.first_touches[-1]["actor"] != actor.key.id
        ):
            self.first_touches.append(
                dict(
                    first=self.tick,
                    team=team,
                    actor=actor.key.id,
                    action=action,
                    values=list(values),
                )
            )
        self.first_touches[-1]["last"] = self.tick
        if action == "release":
            return out
        away = unit(b.x - rival.x, b.y - rival.y)
        # Slide past the nearer edge of the opponent's roller while retaining the outlet.
        left = (-math.sin(rival.yaw), math.cos(rival.yaw))
        side = 1 if (b.x - rival.x) * left[0] + (b.y - rival.y) * left[1] >= 0 else -1
        direction = (
            away
            if action == "shield"
            else unit(0.45 * away[0] + side * left[0], 0.45 * away[1] + side * left[1])
        )
        angle = math.atan2(away[1], away[0]) if action == "shield" else face(b, mate)
        speed = math.hypot(b.vx, b.vy)
        # At the edge of the roller a large pivot outruns the body velocity limit.
        yaw_rate = max(0.6, p["pivot"] * (1 - min(0.86, speed * p["brake"])))
        params = p | dict(tracking=p["follow"], centering=p["grip"], turn=yaw_rate)
        target = (p["cruise"] * direction[0], p["cruise"] * direction[1])
        out[actor.key] = trap(w, actor, angle, target, params)
        if p.get("offer") and action == "sidestep" and option and option["action"] == "dribble":
            # The free teammate widens the passing angle on its existing flank.
            ux, uy = unit(mate.x - b.x, mate.y - b.y)
            sign = 1 if (mate.y - b.y) * ux >= 0 else -1
            x = max(-4.10, min(4.10, mate.x - sign * 0.35 * uy))
            y = max(-2.58, min(2.58, mate.y + sign * 0.35 * ux))
            out[mate.key] = move(mate, x, y, face(mate, b), speed=p["run_speed"])
        self.release_plans.pop(self.tick, None)
        self.choice = None
        if option and option["action"] == "dribble":
            option["until"] = min(option["until"], self.tick + 3)
        return out
