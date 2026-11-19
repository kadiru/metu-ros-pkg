"""Separate learned defensive primitives: physical saves and geometric coverage."""

import json
import math
from dataclasses import asdict
from pathlib import Path
import gymnasium as gym
import numpy as np
from .backend import ReferenceBackend
from .model import Ball, Command, Config, Referee, Robot, RobotKey, World, body_vector, possession
from .env import goal_crossed

GUARD = RobotKey("yellow", 1)
TARGET = RobotKey("blue", 0)


class DefenseMicroEnv(gym.Env):
    metadata = {}

    def __init__(self, cfg=Config(), backend=None, kind="block"):
        if kind not in ("block", "cover"):
            raise ValueError("Unknown defense micro behavior")
        self.cfg, self.backend, self.kind = cfg, backend or ReferenceBackend(cfg), kind
        self.contract = "roboturk." + kind + ".v1"
        self.task_version = kind + ".randomized.v1"
        self.action_space = gym.spaces.Box(-1, 1, (1 if kind == "block" else 2,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (16,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        rng = self.np_random
        sign = int(rng.choice([-1, 1]))
        self.steps = self.dwell = self.contact_checks = self.checks = 0
        self.finished = False
        self.max_error = 0.0
        self.phase, self.frequency = (
            float(rng.uniform(-math.pi, math.pi)),
            float(rng.uniform(0.3, 0.7)),
        )
        self.amplitude = float(rng.uniform(0.35, 0.65))
        if self.kind == "block":
            self.anchor = sign * (self.cfg.length / 2 - 0.18)
            target_y = float(rng.uniform(-0.32, 0.32))
            y = float(rng.uniform(-0.35, 0.35))
            x = self.anchor - sign * float(rng.uniform(1.8, 2.8))
            speed = float(rng.uniform(1.5, 2.5))
            time = abs(self.anchor - sign * self.cfg.possession_x - x) / speed
            b = Ball(x, y, sign * speed, (target_y - y) / time)
            r = Robot(
                GUARD,
                self.anchor,
                float(np.clip(target_y + rng.uniform(-0.25, 0.25), -0.40, 0.40)),
                math.pi if sign == 1 else 0,
            )
            robots = (r,)
        else:
            self.anchor = 0.0
            y = self.amplitude * math.sin(self.phase)
            b = Ball(-sign * 1.8, float(rng.uniform(-0.3, 0.3)))
            r = Robot(
                GUARD,
                float(rng.uniform(-0.4, 0.4)),
                (b.y + y) / 2 + float(rng.uniform(-0.5, 0.5)),
                math.pi if sign == 1 else 0,
            )
            robots = (r, Robot(TARGET, sign * 1.8, y, 0))
        self.world = self.backend.reset(World(0.0, robots, b, Referee.RUNNING, sign))
        return self.observation(), self.info(False, "reset")

    def target(self):
        w, b = self.world, self.world.ball
        if self.kind == "cover":
            t = w.robot(TARGET)
            return ((b.x + t.x) / 2, (b.y + t.y) / 2, t.vx / 2, t.vy / 2)
        r = w.robot(GUARD)
        if possession(w, GUARD, self.cfg) and math.hypot(b.vx, b.vy) < 0.35:
            return (self.anchor, r.y, 0.0, 0.0)
        time = (
            (self.anchor - w.attack_sign * self.cfg.possession_x - b.x) / b.vx
            if abs(b.vx) > 0.05
            else 0
        )
        crossing = b.y + b.vy * max(0.0, min(3.0, time))
        return (self.anchor, float(np.clip(crossing, -0.42, 0.42)), 0.0, 0.0)

    def observation(self):
        w, b, r = self.world, self.world.ball, self.world.robot(GUARD)
        tx, ty, tvx, tvy = self.target()
        sign = w.attack_sign
        o = np.array(
            [
                sign * (tx - r.x),
                sign * (ty - r.y),
                sign * r.vx,
                sign * r.vy,
                sign * tvx,
                sign * tvy,
                sign * (b.x - r.x) / 3,
                sign * (b.y - r.y) / 3,
                sign * b.vx / 3,
                sign * b.vy / 3,
                math.hypot(tx - r.x, ty - r.y),
                float(possession(w, GUARD, self.cfg)),
                sign * r.y,
                sign * b.y,
                self.dwell * self.cfg.dt,
                float(self.kind == "block"),
            ],
            np.float32,
        )
        if not np.all(np.isfinite(o)):
            raise ValueError("Nonfinite defensive observation")
        return o

    def step(self, action):
        if self.finished:
            raise RuntimeError("reset required")
        action = np.asarray(action, dtype=np.float32)
        if action.shape != self.action_space.shape or not np.all(np.isfinite(action)):
            raise ValueError("Invalid defensive action")
        a = np.clip(action, -1, 1)
        w, r = self.world, self.world.robot(GUARD)
        sign = w.attack_sign
        vx, vy = (
            (0.0, sign * float(a[0]) * 1.2)
            if self.kind == "block"
            else (sign * float(a[0]), sign * float(a[1]))
        )
        bx, by = body_vector(vx, vy, r.yaw)
        commands = {GUARD: Command(bx, by, 0, 1)}
        if self.kind == "cover":
            t = w.robot(TARGET)
            target_y = self.amplitude * math.sin(self.phase + self.frequency * w.time)
            vy = self.amplitude * self.frequency * math.cos(
                self.phase + self.frequency * w.time
            ) + 3 * (target_y - t.y)
            commands[TARGET] = Command(0, vy)
        self.world = self.backend.step(commands)
        samples = getattr(self.backend, "last_substep_worlds", [self.world])
        self.steps += 1
        scored = False
        last = w.ball
        for sample in samples:
            scored |= goal_crossed(last, sample.ball, self.cfg, sign)
            last = sample.ball
            self.checks += 1
            self.contact_checks += int(possession(sample, GUARD, self.cfg))
        r, b = self.world.robot(GUARD), self.world.ball
        tx, ty, _, _ = self.target()
        error = math.hypot(tx - r.x, ty - r.y)
        self.max_error = max(self.max_error, error)
        if self.kind == "block":
            stable = (
                all(possession(s, GUARD, self.cfg) for s in samples)
                and math.hypot(b.vx - r.vx, b.vy - r.vy) < 0.2
            )
            needed = 25
        else:
            stable = all(
                math.hypot(
                    (s.ball.x + s.robot(TARGET).x) / 2 - s.robot(GUARD).x,
                    (s.ball.y + s.robot(TARGET).y) / 2 - s.robot(GUARD).y,
                )
                < 0.08
                for s in samples
            )
            needed = 150
        self.dwell = self.dwell + 1 if stable else 0
        collision = self.kind == "cover" and any(
            math.hypot(s.robot(GUARD).x - s.robot(TARGET).x, s.robot(GUARD).y - s.robot(TARGET).y)
            < 2 * self.cfg.robot_radius + 0.01
            for s in samples
        )
        out = (
            abs(r.x) > self.cfg.length / 2 - self.cfg.robot_radius
            or abs(r.y) > self.cfg.width / 2 - self.cfg.robot_radius
            or abs(b.x) > self.cfg.length / 2 + 0.1
            or abs(b.y) > self.cfg.width / 2
        )
        success = self.dwell >= needed and not scored and not collision and not out
        done = success or scored or collision or out
        truncated = self.steps >= 400 and not done
        self.finished = done or truncated
        reason = (
            "saved"
            if success and self.kind == "block"
            else "covered"
            if success
            else "goal_conceded"
            if scored
            else "collision"
            if collision
            else "out"
            if out
            else "timeout"
            if truncated
            else "running"
        )
        reward = (
            30
            if success
            else -25
            if done
            else -0.01 - min(1, error) * 0.1 + (0.05 if stable else 0)
        )
        return (
            self.observation(),
            float(reward),
            bool(done),
            bool(truncated),
            self.info(success, reason),
        )

    def info(self, success, reason):
        return {
            "is_success": bool(success),
            "reason": reason,
            "elapsed_seconds": self.steps * self.cfg.dt,
            "attack_sign": self.world.attack_sign,
            "stable_dwell_seconds": self.dwell * self.cfg.dt,
            "possession_physics_checks": self.contact_checks,
            "physics_checks": self.checks,
            "maximum_target_error": self.max_error,
            "task": self.task_version,
        }

    def close(self):
        self.backend.close()


class DefenseTeacher:
    def action(self, env):
        r = env.world.robot(GUARD)
        tx, ty, tvx, tvy = env.target()
        sign = env.world.attack_sign
        if env.kind == "block":
            return np.array([np.clip(sign * 8 * (ty - r.y) / 1.2, -1, 1)], np.float32)
        return np.clip(
            np.array([sign * (3 * (tx - r.x) + tvx), sign * (3 * (ty - r.y) + tvy)], np.float32),
            -1,
            1,
        )


class DefenseCheckpoint:
    def __init__(self, path, env):
        from stable_baselines3 import PPO

        meta = json.loads(Path(path).with_suffix(".json").read_text())
        if meta["contract"] != env.contract or meta["config"] != asdict(env.cfg):
            raise ValueError("Defense manifest mismatch")
        self.model = PPO.load(path, device="cpu")
        if (
            self.model.observation_space.shape != (16,)
            or self.model.action_space.shape != env.action_space.shape
        ):
            raise ValueError("Defense shape mismatch")

    def action(self, env):
        return self.model.predict(env.observation(), deterministic=True)[0]
