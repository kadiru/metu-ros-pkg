"""Capture, retain through a meaningful carry, then score with an actual kick."""

from dataclasses import replace
import math
from .capture import CaptureEnv, CaptureTeacher
from .dribble import evaluate_dribble
from .env import goal_crossed
from .model import Command, Intent, Skill, body_vector, bounded, possession
from .shoot_contract import SHOT_SPEED, shoot_action, shoot_command, shoot_observation
from gymnasium import spaces
import numpy as np


class ShootTeacher:
    """Demonstrations only; the released checkpoint never calls this controller."""

    def command(self, world, key, intent, cfg):
        r, b = world.robot(key), world.ball
        goal_distance = math.hypot(intent.target[0] - b.x, intent.target[1] - b.y)
        if math.hypot(b.vx, b.vy) > 1.5 and goal_distance < 1.8:
            return Command(kick=SHOT_SPEED)
        if not possession(world, key, cfg):
            return CaptureTeacher().command(world, key, intent, cfg)
        direction = world.direction(key.team)
        bx, by = body_vector(b.x - r.x, b.y - r.y, r.yaw)
        gx, gy = body_vector(intent.target[0] - b.x, intent.target[1] - b.y, r.yaw)
        angle = math.atan2(gy, gx)
        dx, dy = body_vector(
            intent.target[0] - direction * 1.2 - b.x, intent.target[1] - b.y, r.yaw
        )
        if goal_distance < 1.45 and abs(angle) < 0.04 and math.hypot(r.vx, r.vy) < 0.20:
            return Command(kick=SHOT_SPEED)
        omega = max(-0.9, min(0.9, 2.0 * angle))
        distance = max(1e-6, math.hypot(dx, dy))
        speed = min(0.65, 1.4 * distance) * max(0.0, math.cos(angle)) ** 2
        return bounded(
            Command(
                speed * dx / distance + 3 * (bx - cfg.possession_x),
                speed * dy / distance - cfg.possession_x * omega + 3 * by,
                omega,
                1.0,
            ),
            cfg,
        )


class ShootEnv(CaptureEnv):
    task_version = "capture_dribble_shoot.randomized.v1"
    episode_steps = 1200
    minimum_carry = 0.5
    maximum_shot_distance = 1.8

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.skill = Skill.SHOOT
        self.intent = Intent(Skill.SHOOT, (self.cfg.length / 2, 0.0))
        self.observation_space = spaces.Box(-np.inf, np.inf, (17,), dtype=np.float32)
        self.clear_metrics()

    def clear_metrics(self):
        self.acquired = self.shot = False
        self.capture_step = self.shot_step = None
        self.capture_distance = self.initial_target_distance = self.carried_progress = 0.0
        self.possession_checks = self.lost_checks = self.applied_kicks = 0
        self.shot_distance = self.shot_heading_error = self.goal_crossing_y = None
        self.max_mouth_error = 0.0
        self.last_kick = False

    def reset(self, *, seed=None, options=None):
        self.clear_metrics()
        super().reset(seed=seed, options=options)
        direction = int(self.np_random.choice([-1, 1]))
        self.world = replace(self.world, attack_sign=direction)
        self.backend.world = replace(self.backend.world, attack_sign=direction)
        self.intent = Intent(Skill.SHOOT, (direction * self.cfg.length / 2, 0.0))
        self.initial_target_distance = self.target_distance(self.world)
        self.scenario.update(attack_sign=direction, target=list(self.intent.target))
        return self.observation(), self.info(False, "reset")

    def observation(self):
        return shoot_observation(self.world, self.key, self.intent, self.cfg)

    def target_distance(self, world):
        return math.hypot(
            world.ball.x - self.intent.target[0], world.ball.y - self.intent.target[1]
        )

    def info(self, success, reason):
        return {
            **super().info(success, reason),
            "elapsed_seconds": round(self.steps * self.cfg.dt, 3),
            "capture_seconds": None
            if self.capture_step is None
            else self.capture_step * self.cfg.dt,
            "shot_seconds": None if self.shot_step is None else self.shot_step * self.cfg.dt,
            "acquired": self.acquired,
            "shot": self.shot,
            "applied_kicks": self.applied_kicks,
            "target_distance": self.target_distance(self.world),
            "initial_target_distance": self.initial_target_distance,
            "carried_progress": self.carried_progress,
            "shot_distance": self.shot_distance,
            "shot_heading_error": self.shot_heading_error,
            "goal_crossing_y": self.goal_crossing_y,
            "possession_checks": self.possession_checks,
            "lost_possession_checks": self.lost_checks,
            "max_mouth_error": self.max_mouth_error,
        }

    def step(self, action):
        if self.finished:
            raise RuntimeError("reset required")
        previous = self.world
        command = shoot_command(action, self.cfg)
        fire_edge = command.kick > 0 and not self.last_kick
        self.world = self.backend.step({self.key: command})
        self.last_kick = command.kick > 0
        self.steps += 1
        samples = getattr(self.backend, "last_substep_worlds", [self.world])
        events = [e for e in getattr(self.backend, "last_kick_events", []) if e[0] == self.key]
        self.applied_kicks += len(events)
        invalid_shot = False
        if fire_edge and not self.shot:
            self.carried_progress = (
                self.capture_distance - self.target_distance(previous) if self.acquired else 0.0
            )
            valid = (
                self.acquired
                and self.carried_progress >= self.minimum_carry
                and possession(previous, self.key, self.cfg)
                and self.target_distance(previous) <= self.maximum_shot_distance
                and len(events) == 1
            )
            if valid:
                self.shot, self.shot_step = True, self.steps
                self.shot_distance = self.target_distance(events[0][1])
                r, b = previous.robot(self.key), previous.ball
                gx, gy = body_vector(
                    self.intent.target[0] - b.x, self.intent.target[1] - b.y, r.yaw
                )
                self.shot_heading_error = math.atan2(gy, gx)
            else:
                invalid_shot = True
        lost = False
        if not self.shot and not invalid_shot:
            r, b = self.world.robot(self.key), self.world.ball
            held = [possession(w, self.key, self.cfg) and command.dribbler > 0.5 for w in samples]
            stable = all(held) and math.hypot(r.vx - b.vx, r.vy - b.vy) < 0.2
            self.dwell = self.dwell + 1 if stable else 0
            if not self.acquired and self.dwell >= self.hold_steps:
                self.acquired, self.capture_step = True, self.steps
                self.capture_distance = self.target_distance(self.world)
            if self.acquired:
                self.carried_progress = self.capture_distance - self.target_distance(self.world)
                self.possession_checks += len(held)
                self.lost_checks += sum(not h for h in held)
                lost = not all(held)
                for w in samples:
                    r, b = w.robot(self.key), w.ball
                    bx, by = body_vector(b.x - r.x, b.y - r.y, r.yaw)
                    self.max_mouth_error = max(
                        self.max_mouth_error, math.hypot(bx - self.cfg.possession_x, by)
                    )
        scored = False
        last = previous.ball
        for w in samples:
            if goal_crossed(last, w.ball, self.cfg, self.world.direction(self.key.team)):
                scored = True
                line = self.world.direction(self.key.team) * (
                    self.cfg.length / 2 + self.cfg.ball_radius
                )
                alpha = (line - last.x) / (w.ball.x - last.x)
                self.goal_crossing_y = last.y + alpha * (w.ball.y - last.y)
            last = w.ball
        success = scored and self.shot and not invalid_shot
        r, b = self.world.robot(self.key), self.world.ball
        out = (
            abs(r.x) > self.cfg.length / 2 - self.cfg.robot_radius
            or abs(r.y) > self.cfg.width / 2 - self.cfg.robot_radius
            or abs(b.y) > self.cfg.width / 2
            or abs(b.x) > self.cfg.length / 2 + self.cfg.ball_radius
        )
        terminated = success or lost or invalid_shot or out
        truncated = self.steps >= self.episode_steps and not terminated
        self.finished = terminated or truncated
        reward = -0.01
        if not self.shot:
            reward += 3 * (self.potential(self.world) - self.potential(previous))
            if possession(self.world, self.key, self.cfg):
                reward += (
                    8 * (self.target_distance(previous) - self.target_distance(self.world)) + 0.02
                )
        else:
            reward += 2 * (self.target_distance(previous) - self.target_distance(self.world))
        reward += 40 if success else -25 if lost or invalid_shot or out else 0
        reason = (
            "goal"
            if success
            else "invalid_shot"
            if invalid_shot
            else "lost_possession"
            if lost
            else "miss_or_out"
            if out
            else "timeout"
            if truncated
            else "shot"
            if self.shot
            else "running"
        )
        return (
            self.observation(),
            float(reward),
            bool(terminated),
            bool(truncated),
            self.info(success, reason),
        )


def evaluate_shoot(env, policy, episodes=200, seed=250000, recorder=None):
    result = evaluate_dribble(env, policy, episodes, seed, recorder, action_encoder=shoot_action)
    records = result["episodes"]
    result["valid_shot_episodes"] = sum(r["shot"] for r in records)
    result["invalid_shot_episodes"] = sum(r["reason"] == "invalid_shot" for r in records)
    result["mean_shot_distance"] = (
        float(np.mean([r["shot_distance"] for r in records if r["shot"]]))
        if any(r["shot"] for r in records)
        else None
    )
    return result
