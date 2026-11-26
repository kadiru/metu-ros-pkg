"""Choose a post-steal finish, carry or lead pass with a shared team observation."""

import math
from dataclasses import replace
from .carrier_motor import MOTOR, track_roller
from .model import RobotKey, possession, wrap
from .distribution_value import decision
from .takeaway_finish import FinishGame
from .three_match import CFG
from .three_recipient import candidates, choose, distance, move
from .transition_attack import unit


def clearance(x, y, tx, ty, robots, speed):
    dx, dy = unit(tx - x, ty - y)
    length = math.hypot(tx - x, ty - y)
    gap = 1.5
    for q in robots:
        along = (q.x - x) * dx + (q.y - y) * dy
        if -0.04 < along < length + 0.12:
            t = min(0.65, max(0, along / speed))
            offset = (q.x - x) * dy - (q.y - y) * dx
            predicted = offset + t * (q.vx * dy - q.vy * dx)
            gap = min(gap, abs(predicted), abs(offset) + 0.10)
    return gap


def shot_window(x, y, direction, others):
    gx = 4.54 * direction
    choices = []
    for gy in (0, -0.17, 0.17, -0.30, 0.30):
        gap = clearance(x, y, gx, gy, others, 3.05)
        choices.append((min(0.48, gap) - 0.085 * abs(gy), gy, gap))
    _, gy, gap = max(choices, key=lambda a: a[0])
    return gx, gy, gap


def routes(w, actor):
    b = w.ball
    team = actor.key.team
    direction = w.direction(team)
    rivals = [r for r in w.robots if r.key.team != team]
    result = []
    for m in candidates(w, actor):
        other = next(r for r in candidates(w, actor) if r != m)
        best = None
        for fx, sy in (
            (0, 0),
            (0.38, 0),
            (0.60, 0.36),
            (0.60, -0.36),
            (0, 0.62),
            (0, -0.62),
            (-0.35, 0),
        ):
            x = max(-4.13, min(4.13, m.x + fx * direction + 0.12 * m.vx))
            y = max(-2.65, min(2.65, m.y + sy + 0.12 * m.vy))
            length = math.hypot(x - b.x, y - b.y)
            gap = clearance(b.x, b.y, x, y, rivals + [other], 1.85)
            space = min(math.hypot(q.x - x, q.y - y) for q in rivals)
            _, _, shot = shot_window(x, y, direction, rivals + [other])
            turn = abs(wrap(math.atan2(y - b.y, x - b.x) - actor.yaw))
            travel = math.hypot(x - m.x, y - m.y)
            value = (
                1.3 * min(0.65, gap)
                + 0.65 * min(1.0, space)
                + 0.8 * min(0.45, shot)
                + 0.18 * (x - b.x) * direction
                - 0.22 * turn
                - 0.20 * abs(length - 1.35)
                - 1.6 * max(0, 0.85 - length)
                - 0.16 * travel
            )
            row = [gap, space, shot, (x - b.x) * direction, length, turn, travel]
            if best is None or value > best[0]:
                best = (value, m, x, y, row)
        result.append(best)
    return sorted(result, key=lambda v: -v[0])


def carry(w, r, age, can_kick=True):
    b = w.ball
    direction = w.direction(r.key.team)
    rivals = [q for q in w.robots if q.key.team != r.key.team]
    others = [q for q in w.robots if q.key != r.key]
    gx, gy, _ = shot_window(b.x, b.y, direction, others)
    aim = math.atan2(gy - b.y, gx - b.x)
    # Compare short carry directions, including a lateral move past a shot blocker.
    best = None
    for offset in (0, 0.40, -0.40, 0.80, -0.80, 1.20, -1.20, 1.58, -1.58, 2.15, -2.15):
        dx, dy = math.cos(aim + offset), math.sin(aim + offset)
        x, y = b.x + 0.68 * dx, b.y + 0.68 * dy
        gap = clearance(b.x, b.y, x, y, others, 1.5)
        space = min(math.hypot(x - q.x - 0.24 * q.vx, y - q.y - 0.24 * q.vy) for q in rivals)
        aperture = clearance(x, y, gx, gy, others, 3.0)
        edge = min(4.22 - abs(x), 2.73 - abs(y))
        value = (
            1.55 * min(0.60, gap)
            + 0.72 * min(0.82, space)
            + 0.79 * math.cos(offset)
            + 0.70 * min(0.58, aperture)
            + 3.6 * min(0, edge)
        )
        if best is None or value > best[0]:
            best = (value, dx, dy)
    _, dx, dy = best
    nearest = min(rivals, key=lambda q: distance(q, b))
    ex, ey = unit(b.x - nearest.x, b.y - nearest.y)
    danger = max(0, 1 - distance(nearest, b) / 0.52)
    cmd = track_roller(
        w,
        r,
        (1.55 * dx + 0.64 * danger * ex, 1.55 * dy + 0.64 * danger * ey),
        12 * wrap(aim - r.yaw),
        MOTOR,
    )
    ux = math.cos(r.yaw)
    if not can_kick or direction * ux < 0.15:
        return cmd
    crossing = b.y + (gx - b.x) * math.tan(r.yaw)
    lane = clearance(b.x, b.y, gx, crossing, others, 3.05)
    ok = (
        abs(crossing) < 0.24
        and lane > 0.15
        and age >= 3
        and (abs(gx - b.x) < 1.65 or (lane > 0.34 and abs(gx - b.x) < 3.0) or age > 63)
    )
    return replace(cmd, kick=4.0 if ok else 0.0)


class DistributionGame(FinishGame):
    def __init__(self, *args, decision_start=None, incumbent_sample=None, **kwargs):
        self.decision_start = decision_start
        self.incumbent_sample = incumbent_sample
        super().__init__(*args, **kwargs)

    def observe_recovery(self):
        super().observe_recovery()
        c = self.counter
        if not c or "distribution" in c:
            return
        model = self.teams[c["team"]].get("distribution")
        if not model or self.decision_start not in (None, c["sample"]):
            return
        w = self.world
        actor = w.robot(RobotKey(c["team"], c["actor"]))
        options = routes(w, actor)
        obs = c["x"] + [float(v) for item in options for v in item[4]]
        action = 0 if c["sample"] == self.incumbent_sample else decision(model, obs)
        c.update(distribution=action, distribution_obs=obs)
        if action >= 2:
            _, m, x, y, _ = options[action - 2]
            c.update(recipient=m.key.id, target=[x, y], initial_runner=[m.x, m.y])

    def counter_commands(self, out):
        c = self.counter
        if not c or not c.get("distribution"):
            return super().counter_commands(out)
        w = self.world
        b = w.ball
        team = c["team"]
        age = self.tick - c["sample"] // 4
        held = [q for q in w.robots if possession(w, q.key, CFG)]
        actor = w.robot(RobotKey(team, c["actor"]))
        if c["distribution"] >= 2:
            mate = w.robot(RobotKey(team, c["recipient"]))
            x, y = c["target"]
            if (
                self.flight
                and self.flight["team"] == team
                and self.flight["receiver"] == mate.key.id
            ):
                if possession(w, mate.key, CFG):
                    out[mate.key] = carry(w, mate, age, False)
                elif math.hypot(b.vx, b.vy) > 0.18:
                    dx, dy = unit(b.vx, b.vy)
                    along = max(0.025, (x - b.x) * dx + (y - b.y) * dy)
                    out[mate.key] = move(
                        mate,
                        b.x + (along + 0.115) * dx,
                        b.y + (along + 0.115) * dy,
                        math.atan2(-dy, -dx),
                        speed=1.84,
                    )
                return out
        if len(held) != 1 or held[0].key.team != team or self.flight:
            return out
        r = held[0]
        self.owner = r.key
        self.choice = None
        self.release_plans.pop(self.tick, None)
        if c["distribution"] == 1 or r.key != actor.key or age >= 64:
            out[r.key] = carry(w, r, age)
            return out
        dx, dy = unit(x - b.x, y - b.y)
        aim = math.atan2(dy, dx)
        out[mate.key] = move(mate, x + 0.115 * dx, y + 0.115 * dy, math.atan2(-dy, -dx), speed=1.75)
        rivals = [q for q in w.robots if q.key.team != team]
        nearest = min(rivals, key=lambda q: distance(q, b))
        ex, ey = unit(b.x - nearest.x, b.y - nearest.y)
        danger = max(0, 1 - distance(b, nearest) / 0.8)
        cmd = track_roller(
            w, r, (1.20 * danger * ex, 1.20 * danger * ey), 13 * wrap(aim - r.yaw), MOTOR
        )
        mates = candidates(w, r)
        other = next(q for q in mates if q != mate)
        gap = clearance(b.x, b.y, x, y, rivals + [other], 1.85)
        length = math.hypot(x - b.x, y - b.y)
        ready = math.hypot(mate.x - x, mate.y - y) < max(0.22, 0.58 * length)
        fire = (
            ready
            and length > 0.68
            and abs(wrap(aim - r.yaw)) * length < 0.085
            and abs(wrap(aim + math.pi - mate.yaw)) < 0.52
            and gap > 0.105
        )
        self.choice = replace(
            choose(w, r, self.weights[team]), slot=mates.index(mate), receiver=mate.key.id
        )
        if fire:
            self.release_plans[self.tick] = dict(
                target=[x, y],
                center=[x + 0.115 * dx, y + 0.115 * dy],
                runner=mate.key.id,
                heading=[dx, dy],
                clearance=gap,
            )
        out[r.key] = replace(cmd, kick=2.2 if fire else 0)
        return out
