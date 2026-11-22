"""Pass out of a clean steal or finish as the recipient cushions the ball."""

from dataclasses import replace
import math
from .blocked_outlet import finish_motion
from .carrier_motor import track_roller
from .model import possession, wrap
from .takeaway import TakeawayGame, MODES
from .three_match import CFG, face
from .three_recipient import candidates, choose, distance, move
from .transition_attack import unit

EXTRA = (
    "turn on reception",
    "quick outlet",
    "lead and finish",
    "outside escape",
    "controlled approach",
)


def finish(w, r, p, style, age, can_shoot=True):
    b = w.ball
    opponent = min((q for q in w.robots if q.key.team != r.key.team), key=lambda q: distance(b, q))
    aim, _, aligned = finish_motion(w, r, p | dict(shot_clear=0.10), (0, 0))
    ex, ey = unit(b.x - opponent.x, b.y - opponent.y)
    danger = max(0, 1 - distance(opponent, b) / 0.70)
    dx, dy = unit(4.53 * w.direction(r.key.team) - b.x, -b.y)
    ux, uy = unit(r.x - opponent.x, r.y - opponent.y)
    tang = (-uy, ux)
    sign = 1 if (b.x - r.x) * tang[0] + (b.y - r.y) * tang[1] >= 0 else -1
    if style == "outside escape" and danger > 0.25 and age < 26:
        vx, vy = unit(tang[0] * sign + 0.6 * ux, tang[1] * sign + 0.6 * uy)
        aim = math.atan2(vy, vx)
    else:
        speed = 0.46 if style == "turn on reception" else 0.82
        vx, vy = speed * dx + danger * ex, speed * dy + danger * ey
    if abs(b.y) > 2.45:
        vy -= math.copysign(min(0.95, 2 * (abs(b.y) - 2.45)), b.y)
    if abs(b.x) > 4.12 and b.x * vx > 0:
        vx = 0
    omega = max(-6.6, min(6.6, 8.5 * wrap(aim - r.yaw)))
    motor = dict(drive=0.47, step_v=0.32, grip=22, pivot=6.6, cap=1.17)
    cmd = track_roller(w, r, (vx, vy), omega, motor)
    # The physical heading and projected obstacle clearance govern release.
    return replace(cmd, kick=4.0 if can_shoot and aligned else 0)


class OutletGame(TakeawayGame):
    def counter_commands(self, out):
        c = self.counter
        if not c or c["action"] < len(MODES):
            return super().counter_commands(out)
        style = EXTRA[c["action"] - len(MODES)]
        w = self.world
        b = w.ball
        team = c["team"]
        p = self.teams[team]
        held = [r for r in w.robots if possession(w, r.key, CFG)]
        if len(held) != 1 or held[0].key.team != team:
            return out
        actor = held[0]
        age = self.tick - c["sample"] // 4
        if self.flight:
            if self.flight["team"] == team and self.flight["receiver"] == actor.key.id:
                # Continue matching the incoming ball until the measured reception completes.
                out[actor.key] = finish(w, actor, p, style, age, False)
            return out
        if actor.key.id != c["actor"] or style in ("outside escape", "controlled approach"):
            self.owner = actor.key
            self.choice = None
            self.release_plans.pop(self.tick, None)
            out[actor.key] = finish(w, actor, p, style, age)
            return out
        if style == "turn on reception":
            return out
        # Pick a reachable outlet along the current heading. The same actor can
        # finish directly when it already sees a clear shot.
        direct = finish(w, actor, p, style, age)
        if direct.kick:
            self.owner = actor.key
            self.choice = None
            out[actor.key] = direct
            return out
        mates = candidates(w, actor)
        rivals = [r for r in w.robots if r.key.team != team]
        options = []
        for i, m in enumerate(mates):
            t = min(0.40, distance(m, b) / 2.2) if style == "lead and finish" else 0.07
            x, y = m.x + t * m.vx, m.y + t * m.vy
            dx, dy = unit(x - b.x, y - b.y)
            length = math.hypot(x - b.x, y - b.y)
            angle = math.atan2(dy, dx)
            obstacles = rivals + [u for u in mates if u != m]
            gap = min(
                (
                    abs((q.x - b.x) * dy - (q.y - b.y) * dx)
                    for q in obstacles
                    if 0.08 < (q.x - b.x) * dx + (q.y - b.y) * dy < length - 0.10
                ),
                default=1.0,
            )
            value = (
                min(0.7, gap)
                + 0.4 * min(1, distance(m, min(rivals, key=lambda q: distance(m, q))))
                - 0.95 * abs(wrap(angle - actor.yaw))
                - 0.25 * abs(length - 1.25)
            )
            options.append((value, i, angle, x, y, length, gap))
        _, i, angle, x, y, length, gap = max(options)
        mate = mates[i]
        dx, dy = unit(x - b.x, y - b.y)
        aim = (
            max(-4.26, min(4.26, b.x + dx * max(0.84, length + 0.115))),
            max(-2.76, min(2.76, b.y + dy * max(0.84, length + 0.115))),
        )
        out[mate.key] = move(mate, *aim, face(mate, b), speed=1.15)
        gap_to_rival = min(distance(b, q) for q in rivals)
        aligned = abs(wrap(angle - actor.yaw)) * max(0.8, length) < (
            0.14 if gap_to_rival < 0.30 else 0.09
        )
        self.owner = actor.key
        self.choice = replace(choose(w, actor, self.weights[team]), slot=i, receiver=mate.key.id)
        self.release_plans.pop(self.tick, None)
        ex, ey = unit(
            b.x - min(rivals, key=lambda q: distance(q, b)).x,
            b.y - min(rivals, key=lambda q: distance(q, b)).y,
        )
        cmd = track_roller(
            w,
            actor,
            (0.48 * ex, 0.48 * ey),
            7.8 * wrap(angle - actor.yaw),
            dict(drive=0.50, step_v=0.33, grip=22, pivot=6.6, cap=1.18),
        )
        out[actor.key] = replace(cmd, kick=2.2 if aligned and gap > 0.105 else 0)
        return out
