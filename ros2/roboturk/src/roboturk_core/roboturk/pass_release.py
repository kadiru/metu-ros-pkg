"""Reconsider one planned ground pass using only the state at release."""

from dataclasses import replace
import math

import numpy as np

from .counterpass import DistributionGame, carry, clearance, routes
from .model import RobotKey, wrap
from .takeaway import select
from .three_recipient import distance


ACTIONS = ("release", "delay", "switch", "dribble")


def release_features(world, actor, receiver, target, alternate):
    ball = world.ball
    rivals = [r for r in world.robots if r.key.team != actor.key.team]
    x, y = target
    length = max(0.001, math.hypot(x - ball.x, y - ball.y))
    dx, dy = (x - ball.x) / length, (y - ball.y) / length
    cx, cy = x + 0.115 * dx, y + 0.115 * dy
    offset = math.hypot(cx - receiver.x, cy - receiver.y)
    arrival = length / 1.85
    # Estimate the closest rival's time surplus anywhere along the flight.
    margins = []
    for f in (0.2, 0.4, 0.6, 0.8, 1.0):
        t = arrival * f
        for q in rivals:
            lead = min(0.24, t)
            gap = math.hypot(
                ball.x + f * length * dx - q.x - lead * q.vx,
                ball.y + f * length * dy - q.y - lead * q.vy,
            )
            margins.append((gap - 0.13) / 1.75 - max(0, t - lead))
    other = alternate[4]
    return [
        length,
        offset,
        offset / max(0.10, arrival),
        receiver.vx * dx + receiver.vy * dy,
        receiver.vx * dy - receiver.vy * dx,
        abs(wrap(math.atan2(-dy, -dx) - receiver.yaw)),
        abs(wrap(math.atan2(dy, dx) - actor.yaw)) * length,
        min(distance(q, receiver) for q in rivals),
        min(math.hypot(q.x - cx, q.y - cy) for q in rivals),
        min(margins),
        clearance(ball.x, ball.y, x, y, rivals, 1.85),
        min(distance(q, ball) for q in rivals),
        actor.x * world.direction(actor.key.team),
        abs(actor.y),
        other[0],
        other[1],
        other[2],
        other[4],
        other[5],
    ]


def choose_release(model, obs):
    if "examples" not in model:
        return select(model, obs)
    delta = (np.asarray(model["examples"]) - obs) / model["scale"]
    distance2 = np.mean(np.minimum(delta * delta, 9), axis=1)
    nearest = np.argsort(distance2, kind="stable")[: model["neighbors"]]
    weights = np.exp(-distance2[nearest] / model["bandwidth"])
    values = weights @ np.asarray(model["advantages"])[nearest]
    values[0] += model["release_prior"]
    return int(np.argmax(values))


class ReleaseGame(DistributionGame):
    def __init__(self, *args, release_tick=None, **kwargs):
        self.release_tick = release_tick
        self.release_decisions = []
        super().__init__(*args, **kwargs)

    def counter_commands(self, out):
        out = super().counter_commands(out)
        c = self.counter
        plan = self.release_plans.get(self.tick)
        if not c or not plan or c.get("distribution", 0) < 2 or self.flight:
            return out
        team = c["team"]
        model = self.teams[team].get("pass_release")
        scheduled = self.release_tick in (None, self.tick) or "release_review" in c
        if model is None or not scheduled:
            return out
        actor = self.world.robot(RobotKey(team, c["actor"]))
        cmd = out[actor.key]
        if cmd.kick != 2.2 or plan.get("runner") != c["recipient"]:
            return out
        if "release_review" in c:
            if self.tick < c["release_review"]["resume"]:
                self.release_plans.pop(self.tick)
                out[actor.key] = replace(cmd, kick=0)
            return out
        receiver = self.world.robot(RobotKey(team, c["recipient"]))
        options = routes(self.world, actor)
        alternate = next(row for row in options if row[1].key != receiver.key)
        obs = release_features(self.world, actor, receiver, c["target"], alternate)
        action = choose_release(model, obs)
        review = dict(
            tick=self.tick,
            sample=c["sample"],
            team=team,
            actor=actor.key.id,
            receiver=receiver.key.id,
            target=c["target"],
            alternate=alternate[1].key.id,
            action=action,
            x=obs,
            resume=self.tick + (8 if action == 1 else 1),
        )
        c["release_review"] = review
        self.release_decisions.append(review)
        if action == 0:
            return out
        self.release_plans.pop(self.tick)
        self.choice = None
        if action == 3:
            c["distribution"] = 1
            out[actor.key] = carry(self.world, actor, self.tick - c["sample"] // 4)
        else:
            option = (
                alternate if action == 2 else next(row for row in options if row[1] == receiver)
            )
            _, mate, x, y, _ = option
            c.update(recipient=mate.key.id, target=[x, y], initial_runner=[mate.x, mate.y])
            out[actor.key] = replace(cmd, kick=0)
        return out
