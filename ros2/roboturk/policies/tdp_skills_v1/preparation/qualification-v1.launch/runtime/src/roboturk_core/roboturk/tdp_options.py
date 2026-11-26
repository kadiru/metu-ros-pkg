"""Goal-conditioned ball-arrival skills; low-level actions may be neural policies.

The intent is a tactical contract, not a hidden teacher command. Analytic teachers
are used only for explicit baselines and training. Native evidence is sampled at
every backend physics step, including actual applied kicks and goal crossings.
"""

from dataclasses import dataclass
import math
import gymnasium as gym
import numpy as np
from .model import Ball, Command, Config, Referee, Robot, RobotKey, World
from .model import body_vector, possession, wrap, bounded
from .env import goal_crossed

KINDS = ("receive", "redirect", "clear")
CONTRACT = "roboturk.tdp_options.v1"
OBSERVATIONS = 34
SPEED = 1.2
OMEGA = 3.0


@dataclass(frozen=True)
class BallIntent:
    kind: str
    point: tuple[float, float]
    heading: float
    outlet: tuple[float, float]


class BallOptionEnv(gym.Env):
    metadata = {}

    def __init__(self, backend, kind="receive", cfg=Config()):
        if kind not in KINDS:
            raise ValueError(kind)
        self.backend, self.kind, self.cfg = backend, kind, cfg
        self.contract = CONTRACT
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (OBSERVATIONS,), np.float32)
        self.action_space = gym.spaces.Box(-1, 1, (4,), np.float32)
        self.training_seed_base = 5030000 + 200000 * KINDS.index(kind)
        self.reset_index = 0
        self.native_resets = []

    def reset(self, *, seed=None, options=None):
        # PPO resets have an explicit disjoint seed stream, never reserved cases.
        if seed is None:
            seed = self.training_seed_base + self.reset_index
            self.reset_index += 1
        elif seed < 1000000:
            seed = self.training_seed_base + seed
        self.native_resets.append(seed)
        super().reset(seed=seed)
        rng = self.np_random
        self.seed_value = seed
        team = str(rng.choice(["blue", "yellow"]))
        self.actor = RobotKey(team, int(rng.integers(2)))
        self.server = RobotKey(team, 1 - self.actor.id)
        sign = int(rng.choice([-1, 1]))
        attack_sign = sign if team == "blue" else -sign
        self.sign = sign
        self.steps = self.dwell = self.max_dwell = self.checks = self.contact_checks = 0
        self.first_contact = None
        self.ever_contact = False
        self.kicked = self.finished = False
        self.events, self.samples = [], []
        self.applied, self.initial_kicks = [], []
        self.collisions = self.exits = self.opponent_touches = 0
        self.premature = False
        self.pass_speed = float(rng.uniform(1.6, 2.5))
        if self.kind in ("receive", "redirect"):
            px = float(rng.uniform(-0.5, 0.5) if self.kind == "receive" else rng.uniform(1.0, 1.8))
            py = float(rng.uniform(-0.5, 0.5))
            angle = (
                float(rng.uniform(-math.pi, math.pi))
                if self.kind == "receive"
                else float(rng.choice([-1, 1]) * rng.uniform(0.65, 0.95))
            )
            u = np.array([math.cos(angle), math.sin(angle)])
            point = np.array([px, py])
            source = point + float(rng.uniform(1.4, 1.9)) * u
            heading = angle if self.kind == "receive" else math.atan2(-py, 4.5 - px)
            robot = point - self.cfg.possession_x * np.array([math.cos(heading), math.sin(heading)])
            robot += rng.uniform(-0.22, 0.22, 2)
            yaw = heading + float(rng.uniform(-0.35, 0.35))
            server = source + self.cfg.possession_x * u
            robots = [
                Robot(
                    self.actor,
                    sign * robot[0],
                    sign * robot[1],
                    wrap(yaw + (math.pi if sign == -1 else 0)),
                ),
                Robot(
                    self.server,
                    sign * server[0],
                    sign * server[1],
                    wrap(angle + math.pi + (math.pi if sign == -1 else 0)),
                ),
            ]
            b = Ball(sign * source[0], sign * source[1])
            outlet = (sign * 4.5, 0.0)
        else:
            side = int(rng.choice([-1, 1]))
            point = np.array([rng.uniform(-3.25, -2.85), rng.uniform(-0.3, 0.3)])
            outlet = (sign * -0.6, sign * side * 1.7)
            heading = math.atan2(side * 1.7 - point[1], -0.6 - point[0])
            u = np.array([math.cos(heading), math.sin(heading)])
            robot = point - float(rng.uniform(0.27, 0.45)) * u
            robot += rng.uniform(-0.06, 0.06, 2)
            yaw = heading + float(rng.uniform(-0.3, 0.3))
            velocity = -float(rng.uniform(0.15, 0.35)) * u
            b = Ball(sign * point[0], sign * point[1], sign * velocity[0], sign * velocity[1])
            threat = RobotKey("yellow" if team == "blue" else "blue", 0)
            robots = [
                Robot(
                    self.actor,
                    sign * robot[0],
                    sign * robot[1],
                    wrap(yaw + (math.pi if sign == -1 else 0)),
                ),
                Robot(threat, sign * -1.6, sign * -side * 0.4, 0),
            ]
        self.intent = BallIntent(
            self.kind,
            (sign * point[0], sign * point[1]),
            wrap(heading + (math.pi if sign == -1 else 0)),
            outlet,
        )
        self.world = self.backend.reset(World(0, tuple(robots), b, Referee.RUNNING, attack_sign))
        self.initial = self.snapshot(self.world)
        self.initial_ball = (b.x, b.y)
        return self.observation(), self.info(False, "reset")

    def target_pose(self):
        """Measured geometry supplied identically at training and inference."""
        b, intent = self.world.ball, self.intent
        if self.kind == "clear":
            heading = math.atan2(intent.outlet[1] - b.y, intent.outlet[0] - b.x)
            point = (b.x, b.y)
        elif self.kind == "receive" and self.ever_contact:
            heading = intent.heading
            # A bounded receiving region, not an unlimited ball chase.
            point = (
                float(np.clip(b.x, intent.point[0] - 0.6, intent.point[0] + 0.6)),
                float(np.clip(b.y, intent.point[1] - 0.6, intent.point[1] + 0.6)),
            )
        else:
            heading, point = intent.heading, intent.point
        return (
            point[0] - self.cfg.possession_x * math.cos(heading),
            point[1] - self.cfg.possession_x * math.sin(heading),
            heading,
        )

    def observation(self):
        w, cfg = self.world, self.cfg
        r, b = w.robot(self.actor), w.ball
        tx, ty, heading = self.target_pose()
        dx, dy = body_vector(tx - r.x, ty - r.y, r.yaw)
        bx, by = body_vector(b.x - r.x, b.y - r.y, r.yaw)
        bvx, bvy = body_vector(b.vx, b.vy, r.yaw)
        rvx, rvy = body_vector(r.vx, r.vy, r.yaw)
        ox, oy = body_vector(self.intent.outlet[0] - r.x, self.intent.outlet[1] - r.y, r.yaw)
        h = wrap(heading - r.yaw)
        others = [p for p in w.robots if p.key != self.actor]
        nearest = min(others, key=lambda p: math.hypot(p.x - r.x, p.y - r.y))
        nx, ny = body_vector(nearest.x - r.x, nearest.y - r.y, r.yaw)
        result = np.array(
            [
                dx,
                dy,
                math.sin(h),
                math.cos(h),
                bx,
                by,
                bvx / 3,
                bvy / 3,
                rvx / 2,
                rvy / 2,
                r.omega / 3,
                float(possession(w, self.actor, cfg)),
                float(self.ever_contact),
                self.dwell * cfg.dt,
                float(self.kicked),
                ox / 5,
                oy / 5,
                self.sign * r.x / 4.5,
                self.sign * r.y / 3,
                self.sign * b.x / 4.5,
                self.sign * b.y / 3,
                math.hypot(b.vx, b.vy) / 3,
                math.hypot(dx, dy),
                nx / 5,
                ny / 5,
                self.steps * cfg.dt / 8,
                *(float(self.kind == k) for k in KINDS),
                (self.intent.point[0] - r.x) * self.sign / 3,
                (self.intent.point[1] - r.y) * self.sign / 3,
                math.cos(r.yaw) * self.sign,
                math.sin(r.yaw) * self.sign,
                float(w.referee == Referee.RUNNING),
            ],
            np.float32,
        )
        assert result.shape == (OBSERVATIONS,) and np.isfinite(result).all()
        return result

    def command(self, action):
        a = np.asarray(action, np.float32)
        if a.shape != (4,) or not np.isfinite(a).all():
            raise ValueError("Expected four finite option outputs")
        a = np.clip(a, -1, 1)
        fire = self.kind != "receive" and a[3] > 0
        if self.world.referee != Referee.RUNNING:
            return Command()
        return bounded(
            Command(
                float(a[0]) * SPEED,
                float(a[1]) * SPEED,
                float(a[2]) * OMEGA,
                0 if fire else 1,
                3.5 if fire else 0,
            ),
            self.cfg,
        )

    def snapshot(self, w):
        return {
            "time": w.time,
            "ball": [w.ball.x, w.ball.y, w.ball.vx, w.ball.vy],
            "robots": [
                {
                    "team": r.key.team,
                    "id": r.key.id,
                    "pose": [r.x, r.y, r.yaw],
                    "velocity": [r.vx, r.vy, r.omega],
                }
                for r in w.robots
            ],
        }

    def step(self, action):
        if self.finished:
            raise RuntimeError("reset required")
        before = self.world
        commands = {self.actor: self.command(action)}
        if self.kind != "clear":
            commands[self.server] = Command(kick=self.pass_speed if self.steps == 0 else 0)
        self.world = self.backend.step(commands)
        self.steps += 1
        for key, w, speed in getattr(self.backend, "last_kick_events", []):
            event = {
                "type": "kick",
                "step": self.steps,
                "robot": {"team": key.team, "id": key.id},
                "speed": speed,
                "snapshot": self.snapshot(w),
            }
            if key == self.actor:
                if self.first_contact is None:
                    self.first_contact = w.time
                self.applied.append(event)
                self.kicked = True
                self.premature |= self.kind == "clear" and self.max_dwell < 20
            else:
                self.initial_kicks.append(event)
            self.events.append(event)
        samples = getattr(self.backend, "last_substep_worlds", [self.world])
        last = before.ball
        scored = conceded = cleared = False
        all_contact = True
        for w in samples:
            self.checks += 1
            held = possession(w, self.actor, self.cfg)
            self.contact_checks += int(held)
            all_contact &= held
            if held and self.first_contact is None:
                self.first_contact = w.time
            self.ever_contact |= held
            r = w.robot(self.actor)
            self.collisions += int(
                any(math.hypot(r.x - o.x, r.y - o.y) < 0.183 for o in w.robots if o.key != r.key)
            )
            self.opponent_touches += int(
                any(
                    o.key.team != r.key.team and math.hypot(w.ball.x - o.x, w.ball.y - o.y) < 0.116
                    for o in w.robots
                )
            )
            self.exits += int(abs(r.x) > 4.41 or abs(r.y) > 2.91)
            if goal_crossed(last, w.ball, self.cfg, self.sign):
                scored = True
                self.events.append(
                    {
                        "type": "goal",
                        "step": self.steps,
                        "before": [last.x, last.y],
                        "after": [w.ball.x, w.ball.y],
                    }
                )
            conceded |= goal_crossed(last, w.ball, self.cfg, -self.sign)
            if self.kind == "clear" and self.sign * last.x < -1 <= self.sign * w.ball.x:
                frac = (-1 - self.sign * last.x) / (self.sign * w.ball.x - self.sign * last.x)
                y = self.sign * (last.y + frac * (w.ball.y - last.y))
                side = 1 if self.sign * self.intent.outlet[1] > 0 else -1
                cleared = 0.8 <= side * y <= 2.2
                self.events.append(
                    {
                        "type": "clearance_gate",
                        "step": self.steps,
                        "y": y,
                        "valid": bool(cleared),
                        "before": [last.x, last.y],
                        "after": [w.ball.x, w.ball.y],
                    }
                )
            last = w.ball
            # Complete 5ms trace: independent audit does not trust success flags.
            self.samples.append(self.snapshot(w))
        r, b = self.world.robot(self.actor), self.world.ball
        stable = (
            all_contact
            and math.hypot(b.vx - r.vx, b.vy - r.vy) < 0.2
            and math.hypot(b.vx, b.vy) < 0.25
        )
        self.dwell = self.dwell + 1 if stable else 0
        self.max_dwell = max(self.max_dwell, self.dwell)
        first_kick = self.applied[0]["snapshot"]["time"] if self.applied else None
        contact_delay = (
            first_kick - self.first_contact
            if first_kick is not None and self.first_contact is not None
            else None
        )
        if self.kind == "receive":
            success = self.dwell >= 25 and not self.applied and len(self.initial_kicks) == 1
        elif self.kind == "redirect":
            success = (
                scored
                and len(self.applied) == 1
                and len(self.initial_kicks) == 1
                and contact_delay is not None
                and -0.0051 <= contact_delay <= 0.15
            )
        else:
            success = cleared and len(self.applied) == 1 and not self.premature
        ball_out = abs(b.y) > 3.05 or abs(b.x) > 4.6
        failed = bool(
            self.collisions
            or self.exits
            or self.opponent_touches
            or conceded
            or self.premature
            or len(self.applied) > 1
            or (ball_out and not scored)
        )
        success = bool(success and not failed)
        done = success or failed or scored
        truncated = self.steps >= 400 and not done
        self.finished = done or truncated
        reason = (
            "success"
            if success
            else "collision"
            if self.collisions
            else "field_exit"
            if self.exits
            else "opponent_touch"
            if self.opponent_touches
            else "own_goal"
            if conceded
            else "premature_clearance"
            if self.premature
            else "invalid_goal"
            if scored
            else "ball_out"
            if ball_out
            else "timeout"
            if truncated
            else "running"
        )
        error = math.hypot(*self.observation()[:2])
        reward = 30 if success else -25 if done else -0.01 - 0.1 * min(1, error) + 0.1 * stable
        return (
            self.observation(),
            float(reward),
            bool(done),
            bool(truncated),
            self.info(success, reason),
        )

    def info(self, success, reason):
        return {
            "seed": self.seed_value,
            "kind": self.kind,
            "contract": CONTRACT,
            "backend": self.backend.name,
            "backend_contract": getattr(self.backend, "contract", None),
            "is_success": bool(success),
            "reason": reason,
            "elapsed_seconds": self.steps * self.cfg.dt,
            "actor": {"team": self.actor.team, "id": self.actor.id},
            "attack_sign": self.sign,
            "intent": {
                "kind": self.intent.kind,
                "point": self.intent.point,
                "heading": self.intent.heading,
                "outlet": self.intent.outlet,
            },
            "initial": self.initial,
            "physics_checks": self.checks,
            "contact_checks": self.contact_checks,
            "first_contact_time": self.first_contact,
            "maximum_stable_dwell": self.max_dwell * self.cfg.dt,
            "collisions": self.collisions,
            "field_exits": self.exits,
            "opponent_touches": self.opponent_touches,
            "premature_clearance": self.premature,
            "actual_actor_kicks": len(self.applied),
            "actual_server_kicks": len(self.initial_kicks),
            "events": self.events,
            "samples": self.samples,
        }

    def close(self):
        self.backend.close()


class BallOptionTeacher:
    """Training-only controller, independently labeled in every baseline record."""

    def action(self, env):
        r = env.world.robot(env.actor)
        tx, ty, heading = env.target_pose()
        gain = 6.0 if env.kind != "redirect" else 8.0
        vx, vy = body_vector(gain * (tx - r.x), gain * (ty - r.y), r.yaw)
        h = wrap(heading - r.yaw)
        omega = np.clip(6 * h, -1.5, 1.5) if possession(env.world, env.actor, env.cfg) else 7 * h
        fire = False
        if env.kind == "redirect":
            fire = possession(env.world, env.actor, env.cfg) and abs(h) < 0.025
        elif env.kind == "clear":
            fire = env.dwell >= 20 and abs(h) < 0.025
        if env.kicked:
            vx = vy = omega = 0.0
            fire = False
        return np.clip(
            np.array([vx / SPEED, vy / SPEED, omega / OMEGA, 0.8 if fire else -0.8], np.float32),
            -1,
            1,
        )
