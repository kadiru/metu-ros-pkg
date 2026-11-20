"""Continuous symmetric 2v2. Contact and turnovers never terminate a match.

All deployed movement/behavior outputs come from the policy. Referee restarts occur
only after whole-ball goals or out-of-bounds; each is explicit in the event ledger.
"""

from collections import Counter
from dataclasses import replace
import math
import numpy as np
from .model import (
    Ball,
    Command,
    Config,
    Referee,
    Robot,
    RobotKey,
    World,
    body_vector,
    possession,
    bounded,
)
from .capture_contract import capture_observation
from .continuous_features import augment

CONTRACT = "roboturk.continuous_robot.v3"
MODES = ("recover", "dribble", "pass", "shoot", "press", "cover", "support")
OBSERVATIONS, ACTIONS = 117, 35
SPEED, OMEGA = 1.2, 4.0
KEYS = tuple(RobotKey(t, i) for t in ("blue", "yellow") for i in range(2))


def name(k):
    return f"{k.team}{k.id}"


def other_team(team):
    return "yellow" if team == "blue" else "blue"


def ordered(w, k):
    return (
        k,
        RobotKey(k.team, 1 - k.id),
        RobotKey(other_team(k.team), 0),
        RobotKey(other_team(k.team), 1),
    )


class Memory:
    def reset(self, w):
        self.dwell = {k: 0 for k in KEYS}
        self.mode = {k: 0 for k in KEYS}
        self.last_kick = {k: -10.0 for k in KEYS}
        self.last_control = w.time
        self.duration = 45.0
        self.scores = Counter(blue=0, yellow=0)

    def observe(self, samples, kicks, cfg):
        w = samples[-1]
        for k in KEYS:
            r, b = w.robot(k), w.ball
            held = all(
                [key for key in KEYS if possession(sample, key, cfg)] == [k] for sample in samples
            )
            stable = held and math.hypot(r.vx - b.vx, r.vy - b.vy) < 0.25
            self.dwell[k] = self.dwell[k] + 1 if stable else 0
        if any(v >= 15 for v in self.dwell.values()):
            self.last_control = w.time
        for k, _, _ in kicks:
            self.last_kick[k] = w.time


def observation(w, k, memory, cfg=Config()):
    sign, r, b = w.direction(k.team), w.robot(k), w.ball
    keys = ordered(w, k)
    values = []
    for key in keys:
        p = w.robot(key)
        values += [
            sign * p.x / 4.5,
            sign * p.y / 3,
            sign * p.vx / 2,
            sign * p.vy / 2,
            sign * math.cos(p.yaw),
            sign * math.sin(p.yaw),
            p.omega / 4,
            float(possession(w, key, cfg)),
            min(30, memory.dwell[key]) / 15,
        ]
    values += [sign * b.x / 4.5, sign * b.y / 3, sign * b.vx / 4, sign * b.vy / 4]
    values += capture_observation(w, k, cfg).tolist()
    for key in keys[1:]:
        p = w.robot(key)
        x, y = body_vector(p.x - r.x, p.y - r.y, r.yaw)
        values += [x / 5, y / 5, math.hypot(x, y) / 5, math.atan2(y, x) / math.pi]
    for gy in (0, 0.35, -0.35):
        x, y = body_vector(sign * 4.5 - r.x, sign * gy - r.y, r.yaw)
        values += [x / 5, y / 5, math.hypot(x, y) / 5, math.atan2(y, x) / math.pi]
    values += [min(5, w.time - memory.last_kick[key]) / 5 for key in keys[:2]]
    values += [min(5, w.time - memory.last_control) / 5]
    values += [float(memory.mode[k] == i) for i in range(len(MODES))]
    values += [
        max(0, memory.duration - w.time) / 45,
        (memory.scores[k.team] - memory.scores[other_team(k.team)]) / 5,
    ]
    result = augment(np.asarray(values, np.float32))
    if result.shape != (OBSERVATIONS,) or not np.isfinite(result).all():
        raise ValueError(f"Invalid continuous observation {result.shape}")
    return result


def decode(action, w, k, cfg=Config()):
    a = np.asarray(action, np.float32)
    if a.shape != (ACTIONS,) or not np.isfinite(a).all():
        raise ValueError("Expected 35 finite continuous-policy outputs")
    a = np.clip(a, -1, 1)
    mode = int(np.argmax(a[:7]))
    motor = a[7 + 4 * mode : 11 + 4 * mode]
    fire = motor[3] > 0 and mode in (2, 3)
    c = Command(
        float(motor[0]) * SPEED,
        float(motor[1]) * SPEED,
        float(motor[2]) * OMEGA,
        0.0 if fire else 1.0,
        (2.2 if mode == 2 else 3.5) if fire else 0.0,
    )
    if w.referee != Referee.RUNNING or not w.ball.visible or not w.robot(k).visible:
        c = Command()
    return mode, bounded(c, replace(cfg, max_speed=SPEED))


def encode(mode, c):
    a = np.zeros(ACTIONS, np.float32)
    a[:7] = -0.8
    a[mode] = 0.8
    a[7 + 4 * mode : 11 + 4 * mode] = [
        c.vx / SPEED,
        c.vy / SPEED,
        c.omega / OMEGA,
        0.8 if c.kick > 0 else -0.8,
    ]
    return np.clip(a, -1, 1)


def initial_world(rng, sign, time=0.0, possession_team=None):
    # Both teams may attack; no permanently assigned keeper or privileged robot.
    bx, by = float(rng.uniform(-1.0, 1.0)), float(rng.uniform(-0.7, 0.7))
    robots = []
    for team in ("blue", "yellow"):
        s = sign if team == "blue" else -sign
        order = rng.permutation(2)
        for j, i in enumerate(order):
            x = (
                bx - s * float(rng.uniform(0.65, 1.25))
                if j == 0
                else -s * float(rng.uniform(1.5, 2.5))
            )
            y = (
                by + float(rng.uniform(-0.5, 0.5))
                if j == 0
                else float(rng.choice([-1, 1]) * rng.uniform(1.0, 1.8))
            )
            yaw = math.atan2(by - y, bx - x) + float(rng.uniform(-0.6, 0.6))
            robots.append(Robot(RobotKey(team, int(i)), x, y, yaw))
    ball = Ball(bx, by, float(rng.uniform(-0.2, 0.2)), float(rng.uniform(-0.2, 0.2)))
    if possession_team:
        key = next(r.key for r in robots if r.key.team == possession_team)
        r = next(r for r in robots if r.key == key)
        yaw = 0 if (sign if possession_team == "blue" else -sign) == 1 else math.pi
        robots = [replace(p, yaw=yaw) if p.key == key else p for p in robots]
        ball = Ball(r.x + 0.115 * math.cos(yaw), r.y + 0.115 * math.sin(yaw))
    return World(time, tuple(sorted(robots, key=lambda r: r.key)), ball, Referee.RUNNING, sign)


class ContinuousMatch:
    def __init__(self, backend, duration=45.0, cfg=Config(), trace=False):
        self.backend, self.cfg, self.duration, self.trace = backend, cfg, duration, trace
        self.memory = Memory()

    def reset(self, seed):
        self.seed = int(seed)
        self.rng = np.random.default_rng(seed)
        self.sign = int(self.rng.choice([-1, 1]))
        self.start_family = ("loose", "blue_possession", "yellow_possession")[seed % 3]
        team = None if seed % 3 == 0 else ("blue" if seed % 3 == 1 else "yellow")
        self.world = self.backend.reset(initial_world(self.rng, self.sign, possession_team=team))
        self.memory.reset(self.world)
        self.steps = 0
        self.finished = False
        self.reason = "running"
        self.scores = Counter(blue=0, yellow=0)
        self.events = []
        self.trace_rows = []
        self.calls = Counter()
        self.modes = Counter()
        self.physics_checks = 0
        self.contact_checks = 0
        self.contact_events = 0
        self.in_contact = False
        self.outside_checks = 0
        self.possession_seconds = Counter(blue=0.0, yellow=0.0)
        self.contested_seconds = 0.0
        self.distance = Counter()
        self.support_distance = Counter()
        self.cover_distance = Counter()
        self.controlled_carry = Counter()
        self.last_owner = None
        self.current_owner = None
        self.acquisitions = Counter()
        self.turnovers = Counter()
        self.recoveries = Counter()
        self.loose_ticks = 0
        self.was_loose = False
        self.last_pass = None
        self.passes = Counter()
        self.shots = Counter()
        self.receptions = Counter()
        self.pending_restart = None
        self.restarts = 0
        self.last_acquisition_step = -1
        self.total_resets = 1
        self.memory.duration = self.duration
        self.memory.scores = self.scores
        return self.world

    def observations(self):
        return np.stack([observation(self.world, k, self.memory, self.cfg) for k in KEYS])

    def step(self, actions):
        if self.finished:
            raise RuntimeError("Reset required")
        if self.pending_restart:
            raise RuntimeError("Call restart_if_needed before inference")
        previous = self.world
        commands = {}
        for k, a in zip(KEYS, actions, strict=True):
            mode, c = decode(a, previous, k, self.cfg)
            commands[k] = c
            self.memory.mode[k] = mode
            self.calls[name(k)] += 1
            self.modes[k.team + "/" + MODES[mode]] += 1
        self.world = self.backend.step(commands)
        self.steps += 1
        samples = getattr(self.backend, "last_substep_worlds", [self.world])
        kicks = self.backend.last_kick_events
        self.memory.observe(samples, kicks, self.cfg)
        # A loose interval is physical absence of exclusive mouth control, not
        # merely a velocity transient that resets the stable-acquisition clock.
        exclusive = all(sum(possession(w, k, self.cfg) for k in KEYS) == 1 for w in samples)
        self.loose_ticks = 0 if exclusive else self.loose_ticks + 1
        self.was_loose |= self.loose_ticks >= 10
        for k, state, speed in kicks:
            mode = self.memory.mode[k]
            typ = "pass" if mode == 2 else "shot"
            getattr(self, "passes" if typ == "pass" else "shots")[k.team] += 1
            event = {
                "type": typ,
                "step": self.steps,
                "robot": name(k),
                "team": k.team,
                "speed": speed,
                "held": possession(state, k, self.cfg),
                "ball": [state.ball.x, state.ball.y],
                "mode": MODES[mode],
            }
            self.events.append(event)
            self.last_pass = event if typ == "pass" else None
        owner = next((k for k in KEYS if self.memory.dwell[k] >= 15), None)
        self.current_owner = owner
        if self.last_pass and (
            self.steps - self.last_pass["step"] > 250
            or (owner and owner.team != self.last_pass["team"])
            or (
                owner
                and name(owner) == self.last_pass["robot"]
                and self.steps - self.last_pass["step"] >= 15
            )
        ):
            self.last_pass = None
        if owner and (owner != self.last_owner or self.was_loose):
            typ = (
                "turnover"
                if self.last_owner and owner.team != self.last_owner.team
                else "acquisition"
            )
            self.acquisitions[owner.team] += 1
            if typ == "turnover":
                self.turnovers[owner.team] += 1
            recovered = bool(self.was_loose and self.last_owner is not None)
            if recovered:
                self.recoveries[owner.team] += 1
            event = {
                "type": typ,
                "step": self.steps,
                "robot": name(owner),
                "team": owner.team,
                "previous_robot": name(self.last_owner) if self.last_owner else None,
                "dwell": self.memory.dwell[owner],
                "after_loose_ball": recovered,
                "ball": [self.world.ball.x, self.world.ball.y],
            }
            self.events.append(event)
            self.last_acquisition_step = self.steps
            if (
                self.last_pass
                and self.last_pass["team"] == owner.team
                and self.last_pass["robot"] != name(owner)
            ):
                dist = math.dist(self.last_pass["ball"], event["ball"])
                if dist >= 0.5:
                    self.receptions[owner.team] += 1
                    self.events.append(
                        {
                            "type": "reception",
                            **{k: v for k, v in event.items() if k != "type"},
                            "pass_step": self.last_pass["step"],
                            "displacement": dist,
                        }
                    )
                    self.last_pass = None
            self.last_owner = owner
            self.was_loose = False
        prior = previous
        for w in samples:
            self.physics_checks += 1
            distances = [
                math.hypot(a.x - b.x, a.y - b.y)
                for i, a in enumerate(w.robots)
                for b in w.robots[i + 1 :]
            ]
            contact = min(distances) < 2 * self.cfg.robot_radius + 0.003
            self.contact_checks += int(contact)
            self.contact_events += int(contact and not self.in_contact)
            self.in_contact = contact
            outside = any(abs(r.x) > 4.41 or abs(r.y) > 2.91 for r in w.robots)
            self.outside_checks += int(outside)
            holders = [k for k in KEYS if possession(w, k, self.cfg)]
            if len(holders) == 1:
                self.possession_seconds[holders[0].team] += self.cfg.dt / len(samples)
            elif len(holders) > 1:
                self.contested_seconds += self.cfg.dt / len(samples)
            for r in w.robots:
                old = prior.robot(r.key)
                dist = math.hypot(r.x - old.x, r.y - old.y)
                self.distance[name(r.key)] += dist
                if self.memory.mode[r.key] == 6 and not possession(w, r.key, self.cfg):
                    self.support_distance[r.key.team] += dist
                if self.memory.mode[r.key] == 5 and not possession(w, r.key, self.cfg):
                    self.cover_distance[r.key.team] += dist
                if (
                    holders == [r.key]
                    and self.memory.dwell[r.key] >= 15
                    and possession(prior, r.key, self.cfg)
                ):
                    self.controlled_carry[r.key.team] += max(
                        0, w.direction(r.key.team) * (w.ball.x - prior.ball.x)
                    )
            for sign in (-1, 1):
                line = sign * (4.5 + self.cfg.ball_radius)
                if sign * prior.ball.x < sign * line <= sign * w.ball.x:
                    alpha = (line - prior.ball.x) / (w.ball.x - prior.ball.x)
                    y = prior.ball.y + alpha * (w.ball.y - prior.ball.y)
                    if abs(y) < 0.5 - self.cfg.ball_radius and not self.pending_restart:
                        team = "blue" if sign == self.sign else "yellow"
                        self.scores[team] += 1
                        self.events.append(
                            {
                                "type": "goal",
                                "step": self.steps,
                                "team": team,
                                "crossing_y": y,
                                "segment": [[prior.ball.x, prior.ball.y], [w.ball.x, w.ball.y]],
                                "direction": sign,
                            }
                        )
                        self.pending_restart = "goal"
            prior = w
        if not self.pending_restart and (
            abs(self.world.ball.x) > 4.55 or abs(self.world.ball.y) > 3.05
        ):
            self.pending_restart = "ball_out"
            self.events.append({"type": "ball_out", "step": self.steps})
        if any(abs(r.x) > 4.85 or abs(r.y) > 3.35 for r in self.world.robots):
            self.finished = True
            self.reason = "physics_failure"
        if not self.finished and self.steps >= round(self.duration / self.cfg.dt):
            self.finished = True
            self.reason = "time"
        if self.trace and (self.steps % 5 == 0 or kicks or self.pending_restart or self.finished):
            self.trace_rows.append(
                {
                    "step": self.steps,
                    "ball": [
                        self.world.ball.x,
                        self.world.ball.y,
                        self.world.ball.vx,
                        self.world.ball.vy,
                    ],
                    "robots": [
                        [
                            name(r.key),
                            r.x,
                            r.y,
                            r.yaw,
                            self.memory.mode[r.key],
                            self.memory.dwell[r.key],
                        ]
                        for r in self.world.robots
                    ],
                    "score": dict(self.scores),
                }
            )
        return self.world

    def restart_if_needed(self):
        if not self.pending_restart or self.finished:
            return
        reason = self.pending_restart
        self.pending_restart = None
        self.events.append({"type": "referee_restart", "reason": reason, "step": self.steps})
        self.world = self.backend.reset(
            initial_world(self.rng, self.sign, self.steps * self.cfg.dt)
        )
        self.memory.reset(self.world)
        self.current_owner = None
        self.last_owner = None
        self.last_pass = None
        self.loose_ticks = 0
        self.was_loose = False
        self.restarts += 1
        self.total_resets += 1
        self.memory.duration = self.duration
        self.memory.scores = self.scores

    def result(self):
        return {
            "metric_version": 2,
            "seed": self.seed,
            "start": self.start_family,
            "attack_sign": self.sign,
            "steps": self.steps,
            "duration": self.steps * self.cfg.dt,
            "reason": self.reason,
            "score": dict(self.scores),
            "calls": dict(self.calls),
            "mode_ticks": dict(self.modes),
            "physics_checks": self.physics_checks,
            "contact_checks": self.contact_checks,
            "contact_events": self.contact_events,
            "contact_fraction": self.contact_checks / max(1, self.physics_checks),
            "outside_checks": self.outside_checks,
            "possession_seconds": dict(self.possession_seconds),
            "contested_seconds": self.contested_seconds,
            "distance": dict(self.distance),
            "support_distance": dict(self.support_distance),
            "cover_distance": dict(self.cover_distance),
            "controlled_carry": dict(self.controlled_carry),
            "acquisitions": dict(self.acquisitions),
            "turnovers": dict(self.turnovers),
            "recoveries": dict(self.recoveries),
            "passes": dict(self.passes),
            "shots": dict(self.shots),
            "receptions": dict(self.receptions),
            "referee_restarts": self.restarts,
            "backend_resets": self.total_resets,
            "events": self.events,
            "trace": self.trace_rows,
        }


def summary(records):
    n = len(records)
    return {
        "matches": n,
        "completed": sum(r["reason"] == "time" for r in records),
        "unfinished": sum(r["reason"] != "time" for r in records),
        "goals": {t: sum(r["score"].get(t, 0) for r in records) for t in ("blue", "yellow")},
        "wins": {
            t: sum(
                r["reason"] == "time" and r["score"][t] > r["score"][other_team(t)] for r in records
            )
            for t in ("blue", "yellow")
        },
        "draws": sum(
            r["reason"] == "time" and r["score"]["blue"] == r["score"]["yellow"] for r in records
        ),
        "goal_matches": sum(sum(r["score"].values()) > 0 for r in records),
        "turnover_matches": sum(sum(r["turnovers"].values()) > 0 for r in records),
        "acquisition_matches": sum(sum(r["acquisitions"].values()) > 0 for r in records),
        "recovery_matches": sum(sum(r.get("recoveries", {}).values()) > 0 for r in records),
        "passes": sum(sum(r["passes"].values()) for r in records),
        "receptions": sum(sum(r["receptions"].values()) for r in records),
        "mean_contact_fraction": sum(r["contact_fraction"] for r in records) / max(1, n),
        "field_exit_matches": sum(r["outside_checks"] > 0 for r in records),
        "mean_possession_seconds": {
            t: sum(r["possession_seconds"][t] for r in records) / max(1, n)
            for t in ("blue", "yellow")
        },
    }
