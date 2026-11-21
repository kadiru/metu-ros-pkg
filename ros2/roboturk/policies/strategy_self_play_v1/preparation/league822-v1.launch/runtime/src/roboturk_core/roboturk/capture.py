"""Randomized capture curriculum and demonstration controller.

Qualification uses separate seed ranges. The teacher is only for demonstration
collection and comparison; it is never mixed into checkpoint inference.
"""

import math
import gymnasium as gym
import numpy as np
from .backend import ReferenceBackend
from .capture_contract import capture_action, capture_command, capture_observation
from .model import (
    Ball,
    Command,
    Config,
    Intent,
    Referee,
    Robot,
    RobotKey,
    Skill,
    World,
    body_vector,
    bounded,
    possession,
)


class CaptureTeacher:
    def command(self, world, key, intent, cfg):
        r, ball = world.robot(key), world.ball
        bx, by = body_vector(ball.x - r.x, ball.y - r.y, r.yaw)
        bvx, bvy = body_vector(ball.vx, ball.vy, r.yaw)
        angle = math.atan2(by, bx)
        # Face the ball before translating quickly, avoiding accidental rear contacts.
        facing = max(0.0, math.cos(angle)) ** 3
        vx = (3.2 * (bx - cfg.possession_x) + 0.65 * bvx) * facing
        vy = (3.2 * by + 0.65 * bvy) * facing
        scale = max(1.0, math.hypot(vx, vy) / 1.4)
        return bounded(Command(vx / scale, vy / scale, 5.0 * angle, 1.0, 0.0), cfg)


class CaptureEnv(gym.Env):
    metadata = {"render_modes": []}
    task_version = "capture.randomized.v1"

    def __init__(self, cfg=Config(), backend=None, moving_probability=0.5):
        super().__init__()
        self.cfg = cfg
        self.backend = backend if backend is not None else ReferenceBackend(cfg)
        self.skill = Skill.CHASE
        self.key = RobotKey("blue", 0)
        self.intent = Intent(Skill.CHASE, (0.0, 0.0))
        self.action_space = gym.spaces.Box(-1, 1, (4,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (12,), dtype=np.float32)
        self.moving_probability = moving_probability
        self.hold_steps = 20  # 0.4 s at the 50 Hz control rate
        self.finished = True

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        rng = self.np_random
        bx, by = float(rng.uniform(-1.7, 1.7)), float(rng.uniform(-0.9, 0.9))
        for _ in range(100):
            distance, bearing = float(rng.uniform(0.35, 2.0)), float(rng.uniform(-math.pi, math.pi))
            rx, ry = bx + distance * math.cos(bearing), by + distance * math.sin(bearing)
            if abs(rx) < 3.7 and abs(ry) < 2.3:
                break
        yaw = float(rng.uniform(-math.pi, math.pi))
        moving = bool(rng.random() < self.moving_probability)
        speed = float(rng.uniform(0.15, 0.6)) if moving else 0.0
        direction = float(rng.uniform(-math.pi, math.pi))
        initial = World(
            0.0,
            (Robot(self.key, rx, ry, yaw),),
            Ball(bx, by, speed * math.cos(direction), speed * math.sin(direction)),
            Referee.RUNNING,
        )
        self.world = self.backend.reset(initial)
        self.scenario = {
            "moving": moving,
            "initial_distance": distance,
            "ball_speed": speed,
            "initial_heading": yaw,
        }
        self.steps = self.dwell = 0
        self.finished = False
        return self.observation(), self.info(False, "reset")

    def observation(self):
        return capture_observation(self.world, self.key, self.cfg)

    def potential(self, world):
        r, b = world.robot(self.key), world.ball
        bx, by = body_vector(b.x - r.x, b.y - r.y, r.yaw)
        return -math.hypot(bx - self.cfg.possession_x, by) - 0.18 * abs(math.atan2(by, bx))

    def info(self, success, reason):
        return {
            "is_success": bool(success),
            "reason": reason,
            "steps": self.steps,
            "capture_seconds": round(self.steps * self.cfg.dt, 3),
            "hold_steps": self.dwell,
            "backend": self.backend.name,
            "task": self.task_version,
            **self.scenario,
        }

    def step(self, action):
        if self.finished:
            raise RuntimeError("reset required")
        previous = self.world
        command = capture_command(action, self.cfg)
        self.world = self.backend.step({self.key: command})
        self.steps += 1
        r, b = self.world.robot(self.key), self.world.ball
        relative_speed = math.hypot(r.vx - b.vx, r.vy - b.vy)
        stable = (
            possession(self.world, self.key, self.cfg)
            and relative_speed < 0.2
            and command.dribbler > 0.5
        )
        self.dwell = self.dwell + 1 if stable else 0
        success = self.dwell >= self.hold_steps
        out = (
            abs(r.x) > self.cfg.length / 2 - self.cfg.robot_radius
            or abs(r.y) > self.cfg.width / 2 - self.cfg.robot_radius
            or abs(b.x) > self.cfg.length / 2
            or abs(b.y) > self.cfg.width / 2
        )
        terminated = success or out
        truncated = self.steps >= self.cfg.max_steps and not terminated
        self.finished = terminated or truncated
        reward = 3.0 * (self.potential(self.world) - self.potential(previous)) - 0.01
        reward += 0.15 if stable else 0.0
        reward -= 0.0003 * (command.vx**2 + command.vy**2 + command.omega**2)
        reward += 15.0 if success else -10.0 if out else 0.0
        reason = "success" if success else "out" if out else "timeout" if truncated else "running"
        return (
            self.observation(),
            float(reward),
            bool(terminated),
            bool(truncated),
            self.info(success, reason),
        )

    def close(self):
        self.backend.close()


def evaluate_capture(env, policy, episodes=200, seed=50000, recorder=None):
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
                f"Evaluation {i + 1}/{episodes}: {sum(r['is_success'] for r in records)}/{i + 1}",
                flush=True,
            )
    successes = sum(r["is_success"] for r in records)
    p, z, n = successes / episodes, 1.959963984540054, episodes
    center = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return {
        "episodes": records,
        "success_rate": p,
        "successes": successes,
        "wilson_95": [center - half, center + half],
        "mean_success_seconds": float(
            np.mean([r["capture_seconds"] for r in records if r["is_success"]])
        )
        if successes
        else None,
        "stationary": {
            "successes": sum(r["is_success"] for r in records if not r["moving"]),
            "episodes": sum(not r["moving"] for r in records),
        },
        "moving": {
            "successes": sum(r["is_success"] for r in records if r["moving"]),
            "episodes": sum(r["moving"] for r in records),
        },
    }
