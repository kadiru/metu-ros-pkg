"""Capture then carry to a goal-side target, with strict post-capture retention."""

import math
from dataclasses import replace
import numpy as np
from gymnasium import spaces
from .capture import CaptureEnv, CaptureTeacher
from .capture_contract import capture_command
from .dribble_contract import dribble_observation
from .model import Command, Intent, Skill, body_vector, bounded, possession


class DribbleTeacher:
    """Demonstration labels only; not called by a deployed checkpoint."""

    def command(self, world, key, intent, cfg):
        if not possession(world, key, cfg):
            return CaptureTeacher().command(world, key, intent, cfg)
        r, b = world.robot(key), world.ball
        bx, by = body_vector(b.x - r.x, b.y - r.y, r.yaw)
        gx, gy = body_vector(intent.target[0] - b.x, intent.target[1] - b.y, r.yaw)
        distance = max(1e-6, math.hypot(gx, gy))
        angle = math.atan2(gy, gx)
        # Turn about the ball at low angular speed before driving toward the goal.
        omega = max(-0.9, min(0.9, 1.8 * angle)) if distance > 0.08 else 0.0
        speed = min(0.65, 1.4 * distance) * max(0.0, math.cos(angle)) ** 2
        return bounded(
            Command(
                speed * gx / distance + 3.0 * (bx - cfg.possession_x),
                speed * gy / distance - cfg.possession_x * omega + 3.0 * by,
                omega,
                1.0,
            ),
            cfg,
        )


class DribbleEnv(CaptureEnv):
    task_version = "capture_dribble.randomized.v1"
    episode_steps = 1200  # 24 seconds, independent of the generic skill timeout.
    target_radius = 0.18
    arrival_hold_steps = 50

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.skill = Skill.DRIBBLE
        self.intent = Intent(Skill.DRIBBLE, (3.6, 0.0))
        self.observation_space = spaces.Box(-np.inf, np.inf, (16,), dtype=np.float32)
        self.acquired = False
        self.arrival_dwell = self.possession_checks = self.lost_checks = 0
        self.capture_step = None
        self.capture_distance = self.initial_target_distance = 0.0
        self.max_mouth_error = 0.0

    def reset(self, *, seed=None, options=None):
        self.acquired = False
        self.arrival_dwell = self.possession_checks = self.lost_checks = 0
        self.capture_step = None
        self.capture_distance = self.initial_target_distance = self.max_mouth_error = 0.0
        super().reset(seed=seed, options=options)
        direction = int(self.np_random.choice([-1, 1]))
        self.world = replace(self.world, attack_sign=direction)
        # Metadata is part of the backend's state, so read() preserves the chosen side.
        self.backend.world = replace(self.backend.world, attack_sign=direction)
        self.intent = Intent(Skill.DRIBBLE, (direction * 3.6, 0.0))
        self.initial_target_distance = self.target_distance(self.world)
        self.scenario.update(attack_sign=direction, target=list(self.intent.target))
        return self.observation(), self.info(False, "reset")

    def observation(self):
        return dribble_observation(self.world, self.key, self.intent, self.cfg)

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
            "acquired": self.acquired,
            "target_distance": self.target_distance(self.world),
            "initial_target_distance": self.initial_target_distance,
            "carried_progress": self.capture_distance - self.target_distance(self.world)
            if self.acquired
            else 0.0,
            "arrival_hold_steps": self.arrival_dwell,
            "possession_checks": self.possession_checks,
            "lost_possession_checks": self.lost_checks,
            "max_mouth_error": self.max_mouth_error,
        }

    def step(self, action):
        if self.finished:
            raise RuntimeError("reset required")
        previous = self.world
        command = capture_command(action, self.cfg)
        self.world = self.backend.step({self.key: command})
        self.steps += 1
        r, b = self.world.robot(self.key), self.world.ball
        samples = getattr(self.backend, "last_substep_worlds", [self.world])
        held = [possession(w, self.key, self.cfg) and command.dribbler > 0.5 for w in samples]
        stable = all(held) and math.hypot(r.vx - b.vx, r.vy - b.vy) < 0.2
        self.dwell = self.dwell + 1 if stable else 0
        was_acquired = self.acquired
        if not self.acquired and self.dwell >= self.hold_steps:
            self.acquired = True
            self.capture_step = self.steps
            self.capture_distance = self.target_distance(self.world)
        if self.acquired:
            self.possession_checks += len(held)
            self.lost_checks += sum(not h for h in held)
            for w in samples:
                robot = w.robot(self.key)
                bx, by = body_vector(w.ball.x - robot.x, w.ball.y - robot.y, robot.yaw)
                self.max_mouth_error = max(
                    self.max_mouth_error, math.hypot(bx - self.cfg.possession_x, by)
                )
        lost = self.acquired and not all(held)
        arrived = (
            self.acquired
            and not lost
            and self.target_distance(self.world) < self.target_radius
            and math.hypot(b.vx, b.vy) < 0.15
            and math.hypot(r.vx, r.vy) < 0.15
        )
        self.arrival_dwell = self.arrival_dwell + 1 if arrived else 0
        success = self.arrival_dwell >= self.arrival_hold_steps
        out = (
            abs(r.x) > self.cfg.length / 2 - self.cfg.robot_radius
            or abs(r.y) > self.cfg.width / 2 - self.cfg.robot_radius
            or abs(b.x) > self.cfg.length / 2
            or abs(b.y) > self.cfg.width / 2
        )
        terminated = success or lost or out
        truncated = self.steps >= self.episode_steps and not terminated
        self.finished = terminated or truncated
        # Dense progress is paid only while retaining the ball; throwing it cannot earn carry reward.
        reward = 3.0 * (self.potential(self.world) - self.potential(previous)) - 0.01
        if all(held):
            reward += (
                8.0 * (self.target_distance(previous) - self.target_distance(self.world)) + 0.02
            )
        reward += 3.0 if self.acquired and not was_acquired else 0.0
        reward += 20.0 if success else -20.0 if lost or out else 0.0
        reward += 0.08 if arrived else 0.0
        reward -= 0.0003 * (command.vx**2 + command.vy**2 + command.omega**2)
        reason = (
            "success"
            if success
            else "lost_possession"
            if lost
            else "out"
            if out
            else "timeout"
            if truncated
            else "running"
        )
        return (
            self.observation(),
            float(reward),
            bool(terminated),
            bool(truncated),
            self.info(success, reason),
        )


def evaluate_dribble(env, policy, episodes=200, seed=150000, recorder=None):
    from .capture_contract import capture_action

    records = []
    for i in range(episodes):
        env.reset(seed=seed + i)
        if recorder:
            recorder.begin_episode(i, seed + i, env)
        while True:
            command = policy.command(env.world, env.key, env.intent, env.cfg)
            _, _, done, truncated, info = env.step(capture_action(command, env.cfg))
            if recorder:
                recorder.step(env, info)
            if done or truncated:
                records.append({**info, "seed": seed + i})
                break
        if (i + 1) % 20 == 0:
            print(
                f"Dribble evaluation {i + 1}/{episodes}: {sum(r['is_success'] for r in records)}/{i + 1}",
                flush=True,
            )
    successes = sum(r["is_success"] for r in records)
    p, z, n = successes / episodes, 1.959963984540054, episodes
    center = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return {
        "episodes": records,
        "successes": successes,
        "success_rate": p,
        "wilson_95": [center - half, center + half],
        "mean_success_seconds": float(
            np.mean([r["elapsed_seconds"] for r in records if r["is_success"]])
        )
        if successes
        else None,
        "mean_carried_progress": float(
            np.mean([r["carried_progress"] for r in records if r["is_success"]])
        )
        if successes
        else None,
        "possession_loss_episodes": sum(r["lost_possession_checks"] > 0 for r in records),
        "stationary": {
            "successes": sum(r["is_success"] for r in records if not r["moving"]),
            "episodes": sum(not r["moving"] for r in records),
        },
        "moving": {
            "successes": sum(r["is_success"] for r in records if r["moving"]),
            "episodes": sum(r["moving"] for r in records),
        },
        "positive_goal": {
            "successes": sum(r["is_success"] for r in records if r["attack_sign"] == 1),
            "episodes": sum(r["attack_sign"] == 1 for r in records),
        },
        "negative_goal": {
            "successes": sum(r["is_success"] for r in records if r["attack_sign"] == -1),
            "episodes": sum(r["attack_sign"] == -1 for r in records),
        },
    }
