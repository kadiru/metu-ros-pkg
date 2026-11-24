"""Six-robot possession game, with a learned decision at each passing opportunity."""

from collections import Counter
from dataclasses import asdict, replace
import math
from types import SimpleNamespace

import numpy as np
import torch

from .continuous import SPEED, OMEGA
from .touchline import END_LIMIT, SIDE_LIMIT, brake_axis
from .model import body_vector, world_vector
from .model import Ball, Command, Config, Referee, Robot, RobotKey, World, possession, wrap
from .strategy_pass import observation
from .three_recipient import candidates, choose, distance, features, move

KEYS = tuple(RobotKey(team, i) for team in ("blue", "yellow") for i in range(3))
DT = 0.04
CFG = Config(dt=DT)


def face(robot, point):
    return math.atan2(point.y - robot.y, point.x - robot.x)


def layout(rng, team, sign):
    """One defender contests a randomized passing lane. Identities are permuted."""
    direction = sign if team == "blue" else -sign
    x = rng.uniform(-2.8, -1.5)
    center = rng.uniform(-0.45, 0.45)
    positions = [
        (x, center),
        (x + rng.uniform(1.3, 2), center + rng.uniform(0.65, 1.35)),
        (x + rng.uniform(1.3, 2), center - rng.uniform(0.65, 1.35)),
    ]
    marked = positions[rng.integers(1, 3)]
    defenders = [
        ((marked[0] + x) * 0.5, (marked[1] + center) * 0.5 + rng.uniform(-0.09, 0.09)),
        (1.4, rng.uniform(-1.3, 1.3)),
        (3.5, rng.uniform(-0.5, 0.5)),
    ]
    robots = []
    for color in ("blue", "yellow"):
        active = color == team
        ids = rng.permutation(3)
        for j, (rx, ry) in enumerate(positions if active else defenders):
            yaw = (
                (0 if direction == 1 else math.pi)
                if j == 0 and active
                else math.atan2(center - ry, direction * (x - rx))
            )
            robots.append(Robot(RobotKey(color, int(ids[j])), direction * rx, ry, yaw))
    return World(
        0,
        tuple(sorted(robots, key=lambda r: r.key)),
        Ball(direction * (x + 0.115), center),
        Referee.RUNNING,
        sign,
    )


def filter_commands(world, commands, recovering=()):
    """Limit inward velocity along robot pairs and field edges over one control step."""
    velocities = {}
    for r in world.robots:
        c = commands[r.key]
        velocities[r.key] = list(world_vector(c.vx, c.vy, r.yaw))
    raw = {key: vector.copy() for key, vector in velocities.items()}
    pairs = []
    for i, a in enumerate(world.robots):
        for b in world.robots[:i]:
            gap = distance(a, b)
            if gap > 0.33:
                continue
            ux, uy = (a.x - b.x) / max(gap, 1e-9), (a.y - b.y) / max(gap, 1e-9)
            budget = max(0, (gap - 0.195) * 0.8 / DT)
            va, vb = velocities[a.key], velocities[b.key]
            closing = (vb[0] - va[0]) * ux + (vb[1] - va[1]) * uy
            correction = 0.5 * max(0, closing - budget)
            va[0] += correction * ux
            va[1] += correction * uy
            vb[0] -= correction * ux
            vb[1] -= correction * uy
            pairs.append((va, vb, ux, uy, budget))
    scale = 1.0
    for r in world.robots:
        v = velocities[r.key]
        if r.key in recovering:
            v[0] = brake_axis(r.x, v[0], END_LIMIT, DT)
            v[1] = brake_axis(r.y, v[1], SIDE_LIMIT, DT)
        else:
            for j, (pos, bound) in enumerate([(r.x, 4.35), (r.y, 2.85)]):
                v[j] = max(min(0, (-bound - pos) / DT), min(max(0, (bound - pos) / DT), v[j]))
        scale = min(scale, 1.2 / max(1.2, math.hypot(*v)))
    for va, vb, ux, uy, budget in pairs:
        closing = (vb[0] - va[0]) * ux + (vb[1] - va[1]) * uy
        if closing > budget + 1e-8:
            scale = min(scale, budget / closing)
    changed = False
    applied = {}
    for r in world.robots:
        x, y = velocities[r.key]
        x, y = x * scale, y * scale
        changed |= math.hypot(x - raw[r.key][0], y - raw[r.key][1]) > 1e-7
        vx, vy = body_vector(x, y, r.yaw)
        applied[r.key] = replace(commands[r.key], vx=vx, vy=vy)
    return applied, True if changed else None


def snapshot(world):
    return [
        world.ball.x,
        world.ball.y,
        world.ball.vx,
        world.ball.vy,
        *(v for r in world.robots for v in (r.x, r.y, r.yaw)),
    ]


class Game:
    def __init__(self, backend, motor, weights, seed, seconds, sample, render=None, trace=False):
        self.backend, self.motor, self.weights = backend, motor, weights
        self.rng = np.random.default_rng(seed)
        self.choice_rng = np.random.default_rng([seed, 323])
        self.seed, self.seconds, self.sample = seed, seconds, sample
        self.sign = 1 if seed % 2 == 0 else -1
        self.events, self.passes, self.samples = [], [], []
        self.render, self.trace = render, trace
        self.stats, self.score = Counter(), Counter()
        self.shot_clock = True
        self.tick = 0
        self.reset("blue" if seed % 4 < 2 else "yellow", "start")

    def reset(self, team, reason):
        self.world = self.backend.reset(layout(self.rng, team, self.sign))
        self.clock_start = self.tick
        self.owner = self.choice = self.flight = None
        self.dwell = self.flight_hold = 0
        self.possession_team = team
        self.events.append(dict(tick=self.tick, event=reason, team=team))

    def finish_pass(self, result):
        if self.flight is not None:
            self.flight.update(
                result=result, end=self.tick, reward=1 if result == "complete" else -1
            )
            self.flight = None
            self.flight_hold = 0

    def motor_command(self, carrier, mate):
        world = self.world
        # Original pass encoder reads the first other teammate. Present the chosen
        # receiver there and retain all three opponents for contested possession.
        view = replace(world, robots=(mate, *(r for r in world.robots if r.key != mate.key)))
        state = SimpleNamespace(
            world=view, cfg=CFG, memory=SimpleNamespace(dwell={carrier.key: self.dwell * 2})
        )
        x = torch.from_numpy(observation(state, carrier.key))
        with torch.inference_mode():
            a = self.motor.model(x).clamp(-1, 1).tolist()
        fire = a[3] > 0
        return Command(
            a[0] * SPEED, a[1] * SPEED, a[2] * OMEGA, float(not fire), 2.2 if fire else 0
        )

    def commands(self):
        w, ball = self.world, self.world.ball
        held = [r for r in w.robots if possession(w, r.key, CFG)]
        actual = held[0].key if len(held) == 1 else None
        if self.flight is None:
            if self.owner != actual:
                self.owner, self.choice, self.dwell = actual, None, 0
            self.dwell += 1
        carrier = w.robot(self.owner) if self.owner is not None and self.flight is None else None
        if carrier is not None:
            team = carrier.key.team
            if team != self.possession_team:
                self.clock_start, self.possession_team = self.tick, team
            shoot = carrier.x * w.direction(team) > 1.2
            lanes = [features(w, carrier, m) for m in candidates(w, carrier)]
            can_pass = max(row[0] for row in lanes) >= 0.30 and min(row[3] for row in lanes) >= 0.20
            if not shoot and can_pass and self.choice is None:
                self.choice = choose(
                    w, carrier, self.weights[team], self.choice_rng if team in self.sample else None
                )
        flight = self.flight
        commands = {}
        for team in ("blue", "yellow"):
            players = [r for r in w.robots if r.key.team == team]
            direction = w.direction(team)
            nearest = min(players, key=lambda r: distance(r, ball))
            for r in players:
                angle = face(r, ball)
                if carrier is not None and r.key == carrier.key:
                    if shoot:
                        error = wrap(math.atan2(-ball.y, 4.5 * direction - ball.x) - r.yaw)
                        omega = np.clip(error * 2.5, -1.2, 1.2)
                        commands[r.key] = Command(
                            0, -0.115 * omega, omega, 1, 4 if abs(error) < 0.05 else 0
                        )
                    elif self.choice is None:
                        error = wrap(math.atan2(-ball.y, 4.5 * direction - ball.x) - r.yaw)
                        omega = np.clip(error * 2, -1, 1)
                        commands[r.key] = Command(0.32, -0.115 * omega, omega, 1)
                    else:
                        mate = w.robot(RobotKey(team, self.choice.receiver))
                        commands[r.key] = self.motor_command(carrier, mate)
                    continue
                if flight and team == flight["team"] and r.key.id == flight["receiver"]:
                    vx, vy = ball.vx, ball.vy
                    speed = math.hypot(vx, vy)
                    if speed > 0.25 and not held:
                        ux, uy = vx / speed, vy / speed
                        along = max(0.12, (r.x - ball.x) * ux + (r.y - ball.y) * uy)
                        angle = math.atan2(-uy, -ux)
                        x, y = ball.x + along * ux, ball.y + along * uy
                    else:
                        x, y = ball.x - 0.115 * math.cos(angle), ball.y - 0.115 * math.sin(angle)
                elif carrier and team == carrier.key.team:
                    if self.choice is not None and r.key.id == self.choice.receiver:
                        x, y = r.x, r.y
                    else:
                        x = np.clip(carrier.x + direction * 1.65, -3.8, 3.8)
                        y = np.clip(r.y, -2.35, 2.35)
                elif r.key == nearest.key and (flight is None or team != flight["team"]):
                    if carrier is not None:
                        # Cover the route from the carrier to the nearest support player.
                        rivals = [
                            p for p in w.robots if p.key.team != team and p.key != carrier.key
                        ]
                        marked = min(rivals, key=lambda p: distance(r, p))
                        x, y = (marked.x + ball.x) / 2, (marked.y + ball.y) / 2
                    else:
                        x, y = ball.x - 0.115 * math.cos(angle), ball.y - 0.115 * math.sin(angle)
                else:
                    if flight and team == flight["team"]:
                        x, y = r.x + direction * 0.12, r.y
                    else:
                        x = np.clip(ball.x - direction * (3.2 + 0.3 * r.key.id), -3.95, 3.95)
                        y = np.clip(0.4 * ball.y + (r.key.id - 1) * 0.9, -2.6, 2.6)
                commands[r.key] = move(r, x, y, angle)
        return commands

    def run(self):
        for self.tick in range(round(self.seconds / DT)):
            commands, intervention = filter_commands(self.world, self.commands())
            self.stats["safety_ticks"] += int(intervention is not None)
            self.world = self.backend.step(commands)
            for key, state, speed in self.backend.last_kick_events:
                if self.choice and self.owner == key and speed == 2.2:
                    self.flight = {
                        **asdict(self.choice),
                        "tick": self.tick,
                        "team": key.team,
                        "passer": key.id,
                        "ball": [state.ball.x, state.ball.y],
                    }
                    self.passes.append(self.flight)
                    self.flight_hold = 0
                self.events.append(
                    dict(tick=self.tick, event="kick", team=key.team, robot=key.id, speed=speed)
                )
                self.owner = self.choice = None
            for w in self.backend.last_substep_worlds:
                if self.trace:
                    self.samples.append(snapshot(w))
                self.stats["physical_samples"] += 1
                self.stats["contacts"] += int(
                    any(distance(a, b) < 0.18 for i, a in enumerate(w.robots) for b in w.robots[:i])
                )
                self.stats["exits"] += int(
                    any(abs(r.x) > 4.41 or abs(r.y) > 2.91 for r in w.robots)
                )
                if self.flight:
                    f = self.flight
                    mouths = [r.key for r in w.robots if possession(w, r.key, CFG)]
                    intended = RobotKey(f["team"], f["receiver"])
                    other = [
                        k for k in mouths if k != RobotKey(f["team"], f["passer"]) and k != intended
                    ]
                    travel = math.hypot(w.ball.x - f["ball"][0], w.ball.y - f["ball"][1])
                    self.flight_hold = (
                        self.flight_hold + 1 if mouths == [intended] and travel >= 0.55 else 0
                    )
                    if (
                        other
                        or self.tick - f["tick"] >= 4 / DT
                        or self.flight_hold * self.backend.basic_ms >= 200
                    ):
                        self.finish_pass(
                            "intercepted"
                            if other
                            else "complete"
                            if self.flight_hold * self.backend.basic_ms >= 200
                            else "timeout"
                        )
            b = next(
                (
                    s.ball
                    for s in self.backend.last_substep_worlds
                    if abs(s.ball.x) > CFG.ball_exit_x or abs(s.ball.y) > CFG.ball_exit_y
                ),
                None,
            )
            out = b is not None
            if out or (self.shot_clock and (self.tick - self.clock_start + 1) * DT >= 12):
                goal = (
                    out
                    and abs(b.x) > CFG.ball_exit_x
                    and abs(b.y) < CFG.goal_width / 2 - CFG.ball_radius
                )
                if goal:
                    self.score["blue" if b.x * self.sign > 0 else "yellow"] += 1
                self.finish_pass("restart")
                self.reset(
                    "yellow" if self.possession_team == "blue" else "blue",
                    "goal" if goal else "ball out" if out else "shot clock",
                )
            if self.render and self.tick % 2 == 0:
                self.render(self)
        self.finish_pass("end")
        return dict(
            seed=self.seed,
            seconds=self.seconds,
            score=dict(self.score),
            stats=dict(self.stats),
            passes=self.passes,
            events=self.events,
        )
