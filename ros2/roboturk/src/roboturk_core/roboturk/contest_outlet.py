"""Learned contested-ball motor options and off-ball outlet positioning for 3v3."""

from dataclasses import replace
import math

import numpy as np

from .model import Ball, Command, Referee, Robot, RobotKey, World, possession
from .three_match import CFG, Game, face
from .three_recipient import choose, distance, features, move

# Body vx, vy, yaw rate, roller duty. Mirror actions allow either exit flank.
MANEUVERS = np.array(
    [
        [0, 0, 0, 1],
        [-0.23, 0, 0, 1],
        [-0.42, 0, 0, 0],
        [0.08, 0.45, -1.8, 1],
        [0.08, -0.45, 1.8, 1],
        [-0.16, 0.27, -2.8, 1],
        [-0.16, -0.27, 2.8, 1],
        [0.33, 0.31, 2.6, 1],
        [0.33, -0.31, -2.6, 1],
        [-0.04, 0.14, 3.2, 0],
        [-0.04, -0.14, -3.2, 0],
    ]
)
# Candidate locations in the team's attacking frame, measured from the contest.
OFFSETS = [(1.45, 1.15), (1.45, -1.15), (0.35, 1.55), (0.35, -1.55), (-1.2, 0.9), (-1.2, -0.9)]


def initial(rng):
    return {"motor": rng.normal(0, 0.05, (len(MANEUVERS), 5)), "outlet": rng.normal(0, 0.05, 6)}


def sample(scores, rng):
    p = np.exp(scores - max(scores))
    p /= p.sum()
    index = int(np.argmax(scores) if rng is None else rng.choice(len(p), p=p))
    return index, p


def apply_gradient(policy, decisions, reward, rate):
    for head in ("motor", "outlet"):
        gradients = [np.asarray(d["gradient"]) for d in decisions if d["head"] == head]
        if gradients:
            policy[head] += rate * reward * np.mean(gradients, axis=0)
            np.clip(policy[head], -8, 8, out=policy[head])


def duel_layout(seed, sign):
    rng = np.random.default_rng([seed, 738])
    bx, by = rng.uniform(-0.65, 0.65), rng.uniform(-0.45, 0.45)
    angle = rng.uniform(-math.pi, math.pi)
    robots = []
    for team in ("blue", "yellow"):
        direction = sign if team == "blue" else -sign
        yaw = angle if team == "blue" else angle + math.pi
        ids = rng.permutation(3)
        robots.append(
            Robot(
                RobotKey(team, int(ids[0])),
                bx - 0.115 * math.cos(yaw),
                by - 0.115 * math.sin(yaw),
                yaw,
            )
        )
        for i in (1, 2):
            # Spread support across flanks; team directions separate opponents.
            x = bx + direction * rng.uniform(0.9, 2.15)
            y = by + (1 if i == 1 else -1) * rng.uniform(0.85, 1.75)
            robots.append(Robot(RobotKey(team, int(ids[i])), x, y, math.atan2(by - y, bx - x)))
    return World(0, tuple(sorted(robots, key=lambda r: r.key)), Ball(bx, by), Referee.RUNNING, sign)


class ContestGame(Game):
    def __init__(self, *args, policies, exploring=(), duel=False, **kwargs):
        self.policies = policies
        self.exploring = set(exploring)
        self.duel = duel
        self.decisions = []
        self.roles = {}
        self.dual_ticks = 0
        super().__init__(*args, **kwargs)
        self.shot_clock = False

    def reset(self, team, reason):
        super().reset(team, reason)
        if self.duel and reason == "start":
            self.world = self.backend.reset(duel_layout(self.seed, self.sign))
        self.roles.clear()
        self.dual_ticks = 0

    def activate(self, actor, rival):
        w = self.world
        team = actor.key.team
        policy = self.policies[team]
        rng = self.choice_rng if team in self.exploring else None
        # Pick the support robot with the released recipient selector. Its learned
        # spatial policy scores alternate receiving locations and the current spot.
        idx = choose(w, actor, self.weights[team]).receiver
        mate = w.robot(RobotKey(team, idx))
        direction = w.direction(team)
        points = [(mate.x, mate.y)] + [
            (np.clip(w.ball.x + direction * x, -4.0, 4.0), np.clip(w.ball.y + y, -2.65, 2.65))
            for x, y in OFFSETS
        ]
        rows = []
        for x, y in points:
            dest = replace(mate, x=x, y=y)
            f = features(w, actor, dest)
            rows.append([f[0], f[1], f[2], f[4], distance(mate, dest) / 4, f[3]])
        rows = np.asarray(rows)
        k, p = sample(rows @ policy["outlet"], rng)
        start = self.tick
        self.decisions.append(
            dict(
                team=team,
                tick=start,
                head="outlet",
                action=k,
                actor=actor.key.id,
                mate=idx,
                target=list(points[k]),
                gradient=(rows[k] - p @ rows).tolist(),
            )
        )
        self.roles[team] = dict(
            actor=actor.key,
            mate=mate.key,
            point=points[k],
            start=start,
            last_tie=start,
            action=-1,
            change=start,
        )
        self.motor_option(actor, rival)

    def motor_option(self, actor, rival):
        w = self.world
        role = self.roles[actor.key.team]
        direction = w.direction(actor.key.team)
        flank = math.sin(face(actor, w.robot(role["mate"])) - actor.yaw)
        context = np.array(
            [
                1,
                math.cos(actor.yaw) * direction,
                flank,
                math.sin(rival.yaw - actor.yaw),
                min(1, (self.tick - role["start"]) / 35),
            ]
        )
        scores = self.policies[actor.key.team]["motor"] @ context
        k, p = sample(scores, self.choice_rng if actor.key.team in self.exploring else None)
        grad = -p[:, None] * context
        grad[k] += context
        self.decisions.append(
            dict(
                team=actor.key.team,
                head="motor",
                tick=self.tick,
                actor=actor.key.id,
                action=k,
                gradient=grad.tolist(),
            )
        )
        role.update(action=k, change=self.tick + 8)

    def commands(self):
        out = super().commands()
        w = self.world
        held = [r for r in w.robots if possession(w, r.key, CFG)]
        tied = len({r.key.team for r in held}) == 2
        self.dual_ticks = self.dual_ticks + 1 if tied else 0
        for team, policy in self.policies.items():
            if policy is None:
                continue
            role = self.roles.get(team)
            if tied and self.dual_ticks >= 6:
                actor = next(r for r in held if r.key.team == team)
                rival = next(r for r in held if r.key.team != team)
                if role is None or role["actor"] != actor.key or self.tick - role["last_tie"] > 10:
                    self.activate(actor, rival)
                    role = self.roles[team]
                elif self.tick >= role["change"]:
                    self.motor_option(actor, rival)
                role["last_tie"] = self.tick
            if role is None:
                continue
            age = self.tick - role["last_tie"]
            actor = w.robot(role["actor"])
            mine = len(held) == 1 and held[0].key == actor.key
            if self.flight is not None and age > 0:
                self.roles.pop(team)
                continue
            if (age > 8 and not mine) or age > 180:
                self.roles.pop(team)
                continue
            mate = w.robot(role["mate"])
            x, y = role["point"]
            out[mate.key] = move(mate, x, y, face(mate, w.ball))
            if age == 0 or (age <= 3 and not mine):
                vx, vy, omega, roller = MANEUVERS[role["action"]]
                out[actor.key] = Command(vx, vy, omega, roller, 0)
            elif mine:
                # Establish sole possession, then reuse the frozen neural pass
                # motor. The designated outlet is the frozen selector's choice.
                self.owner = actor.key
                self.choice = choose(w, actor, self.weights[team])
                recipient = w.robot(RobotKey(team, self.choice.receiver))
                cmd = self.motor_command(actor, recipient)
                if age < 7 or (age < 32 and distance(mate, replace(mate, x=x, y=y)) > 0.38):
                    cmd = replace(cmd, kick=0, dribbler=1)
                out[actor.key] = cmd
                if recipient.key != mate.key:
                    out[recipient.key] = move(
                        recipient, recipient.x, recipient.y, face(recipient, w.ball)
                    )
        return out
