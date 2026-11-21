"""Ten learned choices with frozen motor policies in continuous native 2v2.

The selector supplies option entry/exit and target parameters. No playbook or
teacher is called here. Targets latch on entry; changing the selected skill ends
the option. The existing joint translation safety filter remains explicit.
"""

from collections import Counter
from dataclasses import replace
import math
from pathlib import Path
import numpy as np
from .model import Ball, Robot, RobotKey, World, Referee, possession, wrap
from .continuous import ContinuousMatch, KEYS, MODES as OLD_MODES, observation, encode, name
from .tdp_options import BallOptionEnv, BallIntent, KINDS

CONTRACT = "roboturk.learned_options.v1"
MODES = (*OLD_MODES, "receive", "redirect", "clear")
OBSERVATIONS, ACTIONS = 162, 16


class FormationBackend:
    """One initial formation per match; all subsequent resets are real referee resets."""

    def __init__(self, backend):
        self.backend = backend
        self.next_reset = None

    def __getattr__(self, name):
        return getattr(self.backend, name)

    def reset(self, world):
        if self.next_reset is not None:
            world, self.next_reset = self.next_reset, None
        return self.backend.reset(world)

    def step(self, commands):
        return self.backend.step(commands)


def formation(seed):
    """Randomized opportunities, not a behavior/role label in observations."""
    from .continuous import initial_world

    rng = np.random.default_rng(seed + 173)
    sign = int(np.random.default_rng(seed).choice([-1, 1]))
    family = ("open", "receive", "redirect", "clear")[seed % 4]
    if family == "open":
        return initial_world(rng, sign), family
    team = str(rng.choice(["blue", "yellow"]))
    enemy = "yellow" if team == "blue" else "blue"
    direction = sign if team == "blue" else -sign
    actor = RobotKey(team, int(rng.integers(2)))
    mate = RobotKey(team, 1 - actor.id)
    if family in ("receive", "redirect"):
        point = np.array(
            [
                rng.uniform(-0.4, 0.4) if family == "receive" else rng.uniform(1, 1.7),
                rng.uniform(-0.4, 0.4),
            ]
        )
        angle = float(
            math.pi + rng.uniform(-0.3, 0.3)
            if family == "receive"
            else rng.choice([-1, 1]) * rng.uniform(0.65, 0.85)
        )
        u = np.array([math.cos(angle), math.sin(angle)])
        source = point + rng.uniform(1.45, 1.85) * u
        p = point + rng.uniform(-0.06, 0.06, 2)
        players = [(actor, *p, angle), (mate, *(source + 0.115 * u), angle + math.pi)]
        ball = Ball(*source)
        # Both opponents act from the first tick; no stationary delivery fixture.
        players += [
            (
                RobotKey(enemy, 0),
                0.5 if family == "receive" else 2.3,
                -1.7 if source[1] > 0 else 1.7,
                0,
            ),
            (RobotKey(enemy, 1), 3.9, float(rng.choice([-1, 1]) * rng.uniform(0.8, 1.3)), math.pi),
        ]
    else:
        side = int(rng.choice([-1, 1]))
        p = np.array([rng.uniform(-3.25, -2.85), rng.uniform(-0.25, 0.25)])
        angle = math.atan2(side * 1.7 - p[1], -0.6 - p[0])
        u = np.array([math.cos(angle), math.sin(angle)])
        players = [
            (actor, *(p - rng.uniform(0.3, 0.4) * u), angle),
            (mate, -0.5, side * 1.7, math.pi),
            (RobotKey(enemy, 0), -1.3, -side * 0.6, math.pi),
            (RobotKey(enemy, 1), 1.5, side * 1.2, math.pi),
        ]
        velocity = -rng.uniform(0.15, 0.3) * u
        ball = Ball(*p, *velocity)
    robots = tuple(
        sorted(
            (
                Robot(
                    k, direction * x, direction * y, wrap(yaw + (math.pi if direction == -1 else 0))
                )
                for k, x, y, yaw in players
            ),
            key=lambda r: r.key,
        )
    )
    ball = replace(
        ball,
        x=direction * ball.x,
        y=direction * ball.y,
        vx=direction * ball.vx,
        vy=direction * ball.vy,
    )
    return World(0, robots, ball, Referee.RUNNING, sign), family


def intent_values(world, key, intent):
    r, sign = world.robot(key), world.direction(key.team)
    return np.array(
        [
            sign * (intent.point[0] - r.x),
            sign * (intent.point[1] - r.y),
            sign * math.cos(intent.heading),
            sign * math.sin(intent.heading),
            sign * intent.outlet[0] / 4.5,
            sign * intent.outlet[1] / 3,
        ],
        np.float32,
    )


def decode_intent(world, key, kind, values):
    v = np.clip(np.asarray(values), -1, 1).tolist()
    r, sign = world.robot(key), world.direction(key.team)
    return BallIntent(
        kind,
        (r.x + sign * v[0], r.y + sign * v[1]),
        math.atan2(sign * v[3], sign * v[2]),
        (sign * 4.5 * v[4], sign * 3 * v[5]),
    )


def snapshot(world):
    return [
        world.time,
        world.ball.x,
        world.ball.y,
        world.ball.vx,
        world.ball.vy,
        *[
            v
            for k in KEYS
            for v in (
                world.robot(k).x,
                world.robot(k).y,
                world.robot(k).yaw,
                world.robot(k).vx,
                world.robot(k).vy,
                world.robot(k).omega,
            )
        ],
    ]


class MotorBank:
    def __init__(self, root):
        from .continuous_training import load as load_base, descriptor
        from .tdp_training import load as load_skill, descriptor as skill_descriptor

        self.root = Path(root)
        source = self.root / "policies/continuous_play_v2/seed611/generation2/policy.zip"
        self.base = load_base(source)
        self.descriptors = {"continuous": descriptor(source, self.base)}
        self.skills = {}
        for kind in KINDS:
            source = self.root / f"policies/tdp_skills_v1/{kind}/policy.zip"
            self.skills[kind] = load_skill(source, BallOptionEnv(None, kind))
            self.descriptors[kind] = skill_descriptor(source, self.skills[kind])

    def base_actions(self, match):
        return self.base.predict(
            np.stack([observation(match.world, k, match.memory) for k in KEYS]), deterministic=True
        )[0]


class StrategyMatch(ContinuousMatch):
    def __init__(self, backend, bank, duration=20, trace=False):
        super().__init__(FormationBackend(backend), duration, trace=trace)
        self.bank = bank

    def clear_options(self):
        self.options = {k: None for k in KEYS}
        self.strategy_modes = {k: 0 for k in KEYS}
        self.last_team_pass = {}
        self.pending_redirect = None
        self.pending_clear = {}

    def reset(self, seed):
        world, family = formation(seed)
        self.backend.next_reset = world
        super().reset(seed)
        self.start_family = family
        self.clear_options()
        self.option_events, self.decision_trace, self.physics_trace = [], [], []
        self.option_counts, self.motor_calls = Counter(), Counter()
        self.initial_snapshot = snapshot(self.world)
        return self.world

    def start_option(self, key, mode, values):
        env = BallOptionEnv(self.backend, MODES[mode], self.cfg)
        env.actor, env.world, env.sign = key, self.world, self.world.direction(key.team)
        env.intent = decode_intent(self.world, key, env.kind, values)
        env.steps = env.dwell = env.max_dwell = 0
        env.kicked = env.ever_contact = False
        env.first_contact = None
        env.start_step = self.steps
        env.notified = False
        self.options[key] = env
        self.option_events.append(
            {
                "type": "option_start",
                "step": self.steps,
                "robot": name(key),
                "kind": env.kind,
                "intent": {
                    "point": env.intent.point,
                    "heading": env.intent.heading,
                    "outlet": env.intent.outlet,
                },
            }
        )

    def observations(self):
        rows = []
        w = self.world
        for k in KEYS:
            base = observation(w, k, self.memory)
            r, b, sign = w.robot(k), w.ball, w.direction(k.team)
            mate = w.robot(RobotKey(k.team, 1 - k.id))
            env = self.options[k]
            values = list(base) + [float(self.strategy_modes[k] == i) for i in range(10)]
            values += list(intent_values(w, k, env.intent)) if env else [0] * 6
            values += (
                [
                    min(8, env.steps * 0.02) / 8,
                    env.dwell * 0.02,
                    float(env.kicked),
                    float(env.ever_contact),
                    min(8, w.time - env.first_contact) / 8 if env.first_contact is not None else 1,
                    env.max_dwell * 0.02,
                ]
                if env
                else [0, 0, 0, 0, 1, 0]
            )
            passed = self.last_team_pass.get(k.team)
            if passed:
                values += [
                    min(5, w.time - passed["time"]) / 5,
                    sign * (passed["ball"][0] - r.x) / 5,
                    sign * (passed["ball"][1] - r.y) / 5,
                    sign * math.cos(passed["yaw"]),
                    sign * math.sin(passed["yaw"]),
                    float(passed["robot"] != name(k)),
                    passed["speed"] / 3,
                ]
            else:
                values += [1, 0, 0, 0, 0, 0, 0]
            speed = math.hypot(b.vx, b.vy)
            along = ((r.x - b.x) * b.vx + (r.y - b.y) * b.vy) / max(0.01, speed**2)
            px, py = b.x + along * b.vx, b.y + along * b.vy
            incoming = math.atan2(b.y - r.y, b.x - r.x)
            goal = math.atan2(-r.y, sign * 4.5 - r.x)
            teammate = math.atan2(mate.y - r.y, mate.x - r.x)
            nearest = min(
                math.hypot(r.x - p.x, r.y - p.y) for p in w.robots if p.key.team != k.team
            )
            values += [
                float(np.clip(sign * (px - r.x), -3, 3)),
                float(np.clip(sign * (py - r.y), -3, 3)),
                float(np.clip(along, -2, 2)),
                speed / 3,
                sign * math.cos(incoming),
                sign * math.sin(incoming),
                sign * math.cos(goal),
                sign * math.sin(goal),
                sign * math.cos(teammate),
                sign * math.sin(teammate),
                nearest / 5,
            ]
            rows.append(values)
        result = np.asarray(rows, np.float32)
        assert result.shape == (4, OBSERVATIONS) and np.isfinite(result).all(), result.shape
        return result

    def step(self, actions):
        a = np.asarray(actions, np.float32)
        if a.shape != (4, ACTIONS) or not np.isfinite(a).all():
            raise ValueError("Expected four finite learned selector actions")
        choices = a[:, :10].argmax(1)
        motors = self.bank.base_actions(self)
        self.motor_calls["continuous_batches"] += 1
        for i, k in enumerate(KEYS):
            mode = int(choices[i])
            if mode != self.strategy_modes[k]:
                if self.options[k]:
                    self.option_events.append(
                        {
                            "type": "option_end",
                            "step": self.steps,
                            "robot": name(k),
                            "kind": self.options[k].kind,
                        }
                    )
                self.options[k] = None
            self.strategy_modes[k] = mode
            self.option_counts[k.team + "/" + MODES[mode]] += 1
            if mode < 7:
                motors[i, :7] = -0.8
                motors[i, mode] = 0.8
            else:
                if self.options[k] is None:
                    self.start_option(k, mode, a[i, 10:])
                env = self.options[k]
                env.world = self.world
                raw = self.bank.skills[env.kind].predict(env.observation(), deterministic=True)[0]
                self.motor_calls[env.kind] += 1
                motors[i] = encode(0 if mode == 7 else 3, env.command(raw))
        before = self.world
        old_events = len(self.events)
        super().step(motors)
        samples = self.backend.last_substep_worlds
        if self.trace:
            self.decision_trace.append([self.steps, *map(int, choices)])
            self.physics_trace.extend(snapshot(w) for w in samples)
        for k, w, speed in self.backend.last_kick_events:
            event = {
                "type": "applied_kick",
                "step": self.steps,
                "time": w.time,
                "robot": name(k),
                "team": k.team,
                "mode": MODES[self.strategy_modes[k]],
                "speed": speed,
                "snapshot": snapshot(w),
            }
            self.option_events.append(event)
            mode = self.strategy_modes[k]
            if mode == 2:
                self.last_team_pass[k.team] = {
                    **event,
                    "ball": [w.ball.x, w.ball.y],
                    "yaw": w.robot(k).yaw,
                }
            env = self.options[k]
            if env:
                first = env.first_contact if env.first_contact is not None else w.time
                passed = self.last_team_pass.get(k.team)
                if (
                    mode == 8
                    and passed
                    and passed["robot"] != name(k)
                    and w.time - passed["time"] < 5
                    and w.time - first <= 0.15
                    and math.dist(passed["ball"], (w.ball.x, w.ball.y)) >= 0.5
                ):
                    self.pending_redirect = {
                        "type": "redirect_kick",
                        "step": self.steps,
                        "robot": name(k),
                        "team": k.team,
                        "pass_step": passed["step"],
                        "delay": w.time - first,
                    }
                    self.option_events.append(self.pending_redirect.copy())
                elif mode == 9 and env.max_dwell >= 20 and env.sign * w.ball.x < -2.2:
                    self.pending_clear[k.team] = {
                        "type": "clearance_kick",
                        "step": self.steps,
                        "robot": name(k),
                        "team": k.team,
                        "outlet": env.intent.outlet,
                        "dwell": env.max_dwell * 0.02,
                    }
                    self.option_events.append(self.pending_clear[k.team].copy())
                env.kicked = True
            if self.pending_redirect and self.pending_redirect["step"] != self.steps:
                self.pending_redirect = None
            for team, clear in list(self.pending_clear.items()):
                if clear["step"] != self.steps:
                    del self.pending_clear[team]
        for k, env in self.options.items():
            if env is None:
                continue
            env.world = self.world
            env.steps += 1
            held = [possession(w, k, self.cfg) for w in samples]
            env.ever_contact |= any(held)
            if env.first_contact is None:
                env.first_contact = next((w.time for w, h in zip(samples, held) if h), None)
            b, r = self.world.ball, self.world.robot(k)
            stable = (
                all(held)
                and math.hypot(b.vx - r.vx, b.vy - r.vy) < 0.2
                and math.hypot(b.vx, b.vy) < 0.25
            )
            env.dwell = env.dwell + 1 if stable else 0
            env.max_dwell = max(env.max_dwell, env.dwell)
            passed = self.last_team_pass.get(k.team)
            if (
                env.kind == "receive"
                and not env.notified
                and env.dwell >= 25
                and passed
                and passed["robot"] != name(k)
                and self.world.time - passed["time"] < 5
            ):
                self.option_events.append(
                    {
                        "type": "receive_stop",
                        "step": self.steps,
                        "robot": name(k),
                        "team": k.team,
                        "pass_step": passed["step"],
                        "dwell": env.dwell * 0.02,
                    }
                )
                env.notified = True
        previous = before
        for world in samples:
            for team, clear in list(self.pending_clear.items()):
                sign = world.direction(team)
                for key in KEYS:
                    if key.team != team and possession(world, key, self.cfg):
                        self.pending_clear.pop(team, None)
                if team not in self.pending_clear:
                    continue
                if sign * previous.ball.x < -1 <= sign * world.ball.x:
                    f = (-sign - previous.ball.x) / (world.ball.x - previous.ball.x)
                    y = sign * (previous.ball.y + f * (world.ball.y - previous.ball.y))
                    side = 1 if sign * clear["outlet"][1] > 0 else -1
                    if 0.8 <= side * y <= 2.2:
                        self.option_events.append(
                            {
                                **clear,
                                "type": "clearance_complete",
                                "kick_step": clear["step"],
                                "step": self.steps,
                                "crossing_y": y,
                            }
                        )
                    del self.pending_clear[team]
            if self.pending_redirect and any(
                possession(world, k, self.cfg)
                for k in KEYS
                if k.team != self.pending_redirect["team"]
            ):
                self.pending_redirect = None
            previous = world
        for event in self.events[old_events:]:
            if (
                event["type"] == "goal"
                and self.pending_redirect
                and event["team"] == self.pending_redirect["team"]
            ):
                self.option_events.append(
                    {
                        **self.pending_redirect,
                        "type": "redirect_goal",
                        "kick_step": self.pending_redirect["step"],
                        "step": self.steps,
                    }
                )
                self.pending_redirect = None
        return self.world

    def restart_if_needed(self):
        old = self.restarts
        super().restart_if_needed()
        if old != self.restarts:
            self.clear_options()
            self.option_events.append(
                {"type": "restart_snapshot", "step": self.steps, "snapshot": snapshot(self.world)}
            )

    def result(self):
        result = super().result()
        result.update(
            strategy_contract=CONTRACT,
            strategy_ticks=dict(self.option_counts),
            motor_calls=dict(self.motor_calls),
            option_events=self.option_events,
            decision_trace=self.decision_trace,
            physics_trace=self.physics_trace,
            initial_snapshot=self.initial_snapshot,
        )
        return result
