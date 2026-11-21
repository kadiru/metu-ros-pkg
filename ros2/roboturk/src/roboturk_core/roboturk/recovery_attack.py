"""Possession-level decision policy for an attack started by a contested recovery."""

from dataclasses import replace
import math

import numpy as np

from .contest_outlet import duel_layout
from .jaw_recovery import JawGame
from .model import Command, RobotKey, possession, wrap
from .three_match import CFG, face
from .three_recipient import candidates, choose, distance, features, move

ACTIONS = ("pass", "dribble", "shoot")
NFEATURES = 11


def scramble(seed, sign):
    """Two opposing mouths start at the ball. A third player may cover a shooting lane."""
    rng = np.random.default_rng([seed, 608])
    w = duel_layout(seed, sign)
    bx = rng.uniform(-1.45, 1.45)
    by = rng.uniform(-0.75, 0.75)
    dx, dy = bx - w.ball.x, by - w.ball.y
    ball = replace(w.ball, x=bx, y=by)
    robots = [replace(r, x=r.x + dx, y=r.y + dy) for r in w.robots]
    for t in ("blue", "yellow"):
        allies = [r for r in robots if r.key.team == t and distance(r, ball) > 0.3]
        defender = allies[int(rng.integers(2))]
        if rng.random() < 0.72:
            goal = -4.5 * w.direction(t)
            k = rng.uniform(0.40, 0.78)
            x = bx + k * (goal - bx)
            y = by * (1 - k) + rng.uniform(-0.18, 0.18)
            robots[robots.index(defender)] = replace(
                defender, x=x, y=y, yaw=math.atan2(by - y, bx - x)
            )
    return replace(w, robots=tuple(robots), ball=ball)


def clearance(ball, x, y, obstacles):
    dx, dy = x - ball.x, y - ball.y
    size = max(1e-8, dx * dx + dy * dy)
    gap = 2.0
    for r in obstacles:
        fraction = ((r.x - ball.x) * dx + (r.y - ball.y) * dy) / size
        if 0.015 < fraction < 1.03:
            gap = min(gap, math.hypot(r.x - ball.x - fraction * dx, r.y - ball.y - fraction * dy))
    return gap


def shooting_lane(w, carrier):
    goal = 4.53 * w.direction(carrier.key.team)
    others = [r for r in w.robots if r.key != carrier.key]
    options = []
    for y in (-0.34, -0.17, 0, 0.17, 0.34):
        gap = clearance(w.ball, goal, y, others)
        options.append((min(0.5, gap) - 0.055 * abs(y), gap, y))
    _, gap, y = max(options)
    return goal, y, gap


def situation(w, actor, choice):
    goal, y, gap = shooting_lane(w, actor)
    direction = w.direction(actor.key.team)
    mate = w.robot(RobotKey(actor.key.team, choice.receiver))
    f = features(w, actor, mate)
    _, _, mate_gap = shooting_lane(replace(w, ball=replace(w.ball, x=mate.x, y=mate.y)), mate)
    pressure = min(distance(actor, r) for r in w.robots if r.key.team != actor.key.team)
    alignment = math.cos(math.atan2(y - w.ball.y, goal - w.ball.x) - actor.yaw)
    return np.array(
        [
            1,
            actor.x * direction / 4.5,
            abs(actor.y) / 3,
            min(1, gap / 0.33),
            min(1, pressure),
            alignment,
            f[0],
            f[2],
            min(1, mate_gap / 0.33),
            math.cos(face(actor, mate) - actor.yaw),
            float(gap > 0.14),
        ]
    )


def outlet(w, actor, weights):
    original = choose(w, actor, weights)
    mates = candidates(w, actor)
    rows = np.array(original.features)
    scores = rows @ weights
    for i, m in enumerate(mates):
        _, _, lane = shooting_lane(replace(w, ball=replace(w.ball, x=m.x, y=m.y)), m)
        scores[i] += 2.4 * rows[i, 2] + 1.5 * min(0.4, lane) - 1.3 * rows[i, 3] + 0.8 * rows[i, 4]
    j = int(np.argmax(scores))
    probability = float(1 / (1 + np.exp(np.clip(scores[0] - scores[1], -40, 40))))
    return replace(original, receiver=mates[j].key.id, slot=j, probability=probability)


def pivot_pass(w, actor, mate):
    # Aim for the mouth that the receiver presents toward the ball.
    target = face(actor, mate)
    err = wrap(target - actor.yaw)
    omega = float(np.clip(5.5 * err, -3.5, 3.5))
    return Command(0, -0.115 * omega, omega, 1, 2.2 if abs(err) < 0.035 else 0)


def carry(w, actor):
    direction = w.direction(actor.key.team)
    goal, gy, _ = shooting_lane(w, actor)
    dx, dy = goal - actor.x, gy - actor.y
    heading = math.atan2(dy, dx)
    others = [r for r in w.robots if r.key != actor.key]
    options = []
    for offset in (0, -0.5, 0.5, -0.9, 0.9):
        angle = heading + offset
        x = np.clip(actor.x + 1.1 * math.cos(angle), -4.1, 4.1)
        y = np.clip(actor.y + 1.1 * math.sin(angle), -2.65, 2.65)
        gap = clearance(w.ball, x, y, others)
        line = clearance(replace(w.ball, x=x, y=y), goal, gy, others)
        options.append(
            (
                min(0.45, gap)
                + 0.7 * min(0.50, line)
                + 0.15 * direction * (x - actor.x)
                - 0.03 * abs(offset),
                angle,
            )
        )
    err = wrap(max(options)[1] - actor.yaw)
    omega = float(np.clip(3.8 * err, -2.8, 2.8))
    return Command(0.62 * max(0, math.cos(err)) ** 3, -0.115 * omega, omega, 1, 0)


class PossessionEnded(Exception):
    """End a training rollout after its scoring opportunity has been decided."""


class AttackGame(JawGame):
    def __init__(
        self,
        *args,
        strategies,
        fixture,
        epsilon=0,
        learner=None,
        refined=False,
        temperature=0.55,
        early_stop=False,
        **kwargs,
    ):
        self.strategies = strategies
        self.fixture = fixture
        self.early_stop = early_stop
        self.epsilon = epsilon
        self.learner = learner
        self.refined = refined
        self.temperature = temperature
        self.options = []
        self.attack_rng = np.random.default_rng([args[3], 5147])
        super().__init__(*args, **kwargs)

    def reset(self, team, reason):
        super().reset(team, reason)
        self.terminal_restart = reason != "start"
        if reason == "start" and self.fixture:
            self.world = self.backend.reset(scramble(self.seed, self.sign))
        self.contestants = set()
        self.tie_count = self.sole_count = 0
        self.last_tie = -1000
        self.last_key = None
        self.active = None
        self.option = None
        self.skip_frames = True

    def observe_recovery(self):
        if self.skip_frames:
            self.skip_frames = False
            return
        for j, w in enumerate(self.backend.last_substep_worlds):
            sample = (self.tick - 1) * 4 + j
            held = [r.key for r in w.robots if possession(w, r.key, CFG)]
            if self.active and (
                any(k.team != self.active[0] for k in held) or sample - self.active[1] > 1800
            ):
                self.active = self.option = None
            if len({k.team for k in held}) == 2:
                self.tie_count += 1
                self.last_tie = sample
                if self.tie_count == 20 and not self.contestants:
                    self.contestants = set(held)
                self.last_key = None
                self.sole_count = 0
                continue
            self.tie_count = 0
            k = held[0] if len(held) == 1 else None
            self.sole_count = (
                self.sole_count + 1 if k is not None and k == self.last_key else int(k is not None)
            )
            self.last_key = k
            if sample - self.last_tie > 300:
                self.contestants.clear()
            if self.sole_count == 20 and self.contestants:
                if k in self.contestants and self.strategies[k.team] is not None:
                    self.active = (k.team, sample)
                    self.option = None
                self.contestants.clear()

    def run(self):
        try:
            return super().run()
        except PossessionEnded:
            self.finish_pass("end")
            return dict(
                seed=self.seed,
                seconds=self.tick * CFG.dt,
                score=dict(self.score),
                stats=dict(self.stats),
                passes=self.passes,
                events=self.events,
            )

    def commands(self):
        if self.early_stop and self.terminal_restart:
            raise PossessionEnded
        out = super().commands()
        previous = self.active
        self.observe_recovery()
        if self.early_stop and previous and not self.active:
            raise PossessionEnded
        if not self.active:
            return out
        w = self.world
        team, recovery = self.active
        held = [r for r in w.robots if possession(w, r.key, CFG)]
        if self.flight is not None or len(held) != 1 or held[0].key.team != team:
            self.option = None
            return out
        actor = held[0]
        if (
            self.option is None
            or self.option["actor"] != actor.key.id
            or self.tick >= self.option["until"]
        ):
            choice = (outlet if self.refined else choose)(w, actor, self.weights[team])
            x = situation(w, actor, choice)
            if self.refined:
                earlier = [o for o in self.options if (o["team"], o["recovery"]) == self.active]
                received = any(
                    p["team"] == team and p["result"] == "complete" and 4 * p["tick"] >= recovery
                    for p in self.passes
                )
                x = np.r_[
                    x, float(received), min(1, sum(o["action"] == "dribble" for o in earlier) / 2)
                ]
            q = np.asarray(self.strategies[team]) @ x
            if team == self.learner and self.attack_rng.random() < self.epsilon:
                a = int(self.attack_rng.integers(3))
            elif self.refined and self.temperature > 0:
                prob = np.exp((q - q.max()) / self.temperature)
                a = int(self.attack_rng.choice(3, p=prob / prob.sum()))
            else:
                a = int(np.argmax(q))
            self.option = dict(
                team=team,
                recovery=recovery,
                tick=self.tick,
                actor=actor.key.id,
                action=ACTIONS[a],
                x=x.tolist(),
                receiver=choice.receiver,
                until=self.tick + (22 if a == 1 else 88),
            )
            self.options.append(self.option)
            self.target_choice = choice
        action = self.option["action"]
        self.owner = actor.key
        self.choice = None
        if action == "pass":
            self.choice = self.target_choice
            mate = w.robot(RobotKey(team, self.choice.receiver))
            out[mate.key] = move(mate, mate.x, mate.y, face(mate, w.ball))
            out[actor.key] = (
                pivot_pass(w, actor, mate) if self.refined else self.motor_command(actor, mate)
            )
        elif action == "shoot":
            gx, gy, _ = shooting_lane(w, actor)
            err = wrap(math.atan2(gy - w.ball.y, gx - w.ball.x) - actor.yaw)
            omega = float(np.clip(4 * err, -3, 3))
            out[actor.key] = Command(0, -0.115 * omega, omega, 1, 4 if abs(err) < 0.025 else 0)
        else:
            out[actor.key] = carry(w, actor)
        return out
