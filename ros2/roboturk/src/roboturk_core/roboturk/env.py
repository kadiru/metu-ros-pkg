"""Seeded single-skill curriculum with the same observation ABI used by team play."""

import math

import gymnasium as gym
import numpy as np

from .backend import ReferenceBackend
from .behavior import AnalyticPolicy
from .contract import decode_action, observation_size, observe
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
    constrain_skill,
    possession,
)


def goal_crossed(previous: Ball, current: Ball, cfg: Config, sign=1) -> bool:
    line = cfg.ball_exit_x  # whole ball must cross the line
    before, after = sign * previous.x, sign * current.x
    if not before <= line < after:
        return False
    t = (line - before) / (after - before)
    y = previous.y + t * (current.y - previous.y)
    return abs(y) + cfg.ball_radius < cfg.goal_width / 2


class SkillEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, skill=Skill.CHASE, cfg=Config(), backend=None):
        super().__init__()
        self.skill, self.cfg = Skill(skill), cfg
        if self.skill == Skill.HOLD:
            raise ValueError("HOLD is a runtime command, not a curriculum task")
        self.backend = backend if backend is not None else ReferenceBackend(cfg)
        self.action_space = gym.spaces.Box(-1.0, 1.0, (5,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(
            -np.inf, np.inf, (observation_size(cfg),), dtype=np.float32
        )
        self.key, self.receiver = RobotKey("blue", 0), RobotKey("blue", 1)
        self.baseline = AnalyticPolicy()
        self.world = None
        self.finished = True

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        c, rng = self.cfg, self.np_random
        jitter = float(rng.uniform(-0.2, 0.2))
        active = Robot(self.key, -0.8, jitter)
        teammate = Robot(self.receiver, 1.5, 0.0, math.pi)
        ball = Ball(0.0, 0.0)
        target = (1.2, float(rng.uniform(-0.4, 0.4)))
        if self.skill == Skill.CHASE:
            ball = Ball(float(rng.uniform(0.0, 0.6)), jitter)
            target = (c.length / 2, 0.0)
        elif self.skill in (Skill.DRIBBLE, Skill.PASS, Skill.SHOOT):
            active = Robot(self.key, 0.0 if self.skill != Skill.SHOOT else 2.0, jitter)
            ball = Ball(active.x + c.possession_x, active.y)
            if self.skill == Skill.PASS:
                target = (teammate.x, teammate.y)
            if self.skill == Skill.SHOOT:
                target = (c.length / 2, 0.0)
        elif self.skill == Skill.RECEIVE:
            active = Robot(self.key, 1.0, jitter, math.pi)
            teammate = Robot(self.receiver, -1.0, 1.0)
            ball = Ball(0.0, jitter, float(rng.uniform(1.0, 1.5)), 0.0)
            target = (active.x, active.y)
        self.intent = Intent(
            self.skill, target, self.receiver if self.skill == Skill.PASS else None
        )
        world = World(0.0, (active, teammate), ball, Referee.RUNNING)
        self.world = self.backend.reset(world)
        self.steps = self.dwell = 0
        self.pass_released = False
        self.finished = False
        return observe(self.world, self.key, self.intent, c), self._info(False, "reset")

    def _potential(self, world):
        r, b = world.robot(self.key), world.ball
        if self.skill in (Skill.CHASE, Skill.RECEIVE):
            return -math.hypot(r.x - b.x, r.y - b.y)
        entity = r if self.skill == Skill.MOVE else b
        return -math.hypot(entity.x - self.intent.target[0], entity.y - self.intent.target[1])

    def _info(self, success, reason):
        return {
            "is_success": bool(success),
            "reason": reason,
            "backend": self.backend.name,
            "skill": self.skill.value,
            "steps": self.steps,
        }

    def step(self, action):
        if self.finished:
            raise RuntimeError("Call reset before step or after episode completion")
        cfg, previous = self.cfg, self.world
        command = decode_action(action, cfg)
        # A skill never receives unrelated actuators. Inference uses the same restriction below.
        command = constrain_skill(command, self.skill)
        if self.skill == Skill.PASS:
            if self.pass_released:
                command = Command()
            elif command.kick > 0 and possession(previous, self.key, cfg):
                self.pass_released = True
        receiver_command = Command()
        if self.skill == Skill.PASS:
            rr = previous.robot(self.receiver)
            receiver_command = self.baseline.command(
                previous, self.receiver, Intent(Skill.RECEIVE, (rr.x, rr.y)), cfg
            )
        self.world = self.backend.step({self.key: command, self.receiver: receiver_command})
        self.steps += 1
        r, b = self.world.robot(self.key), self.world.ball
        held = possession(self.world, self.key, cfg)
        success_now = False
        if self.skill == Skill.MOVE:
            success_now = (
                math.hypot(r.x - self.intent.target[0], r.y - self.intent.target[1]) < 0.08
            )
        elif self.skill in (Skill.CHASE, Skill.RECEIVE):
            success_now = held and math.hypot(b.vx - r.vx, b.vy - r.vy) < 0.5
        elif self.skill == Skill.DRIBBLE:
            success_now = (
                held and math.hypot(b.x - self.intent.target[0], b.y - self.intent.target[1]) < 0.12
            )
        elif self.skill == Skill.PASS:
            rr = self.world.robot(self.receiver)
            success_now = (
                possession(self.world, self.receiver, cfg)
                and math.hypot(b.vx - rr.vx, b.vy - rr.vy) < 0.5
            )
        elif self.skill == Skill.SHOOT:
            success_now = goal_crossed(previous.ball, b, cfg)
        self.dwell = self.dwell + 1 if success_now else 0
        success = success_now if self.skill == Skill.SHOOT else self.dwell >= 3
        collision = any(
            math.hypot(r.x - n.x, r.y - n.y) < 2 * cfg.robot_radius
            for n in self.world.robots
            if n.key != self.key
        )
        out = (
            abs(r.x) > cfg.length / 2 - cfg.robot_radius
            or abs(r.y) > cfg.width / 2 - cfg.robot_radius
            or abs(b.y) > cfg.width / 2
            or abs(b.x) > cfg.ball_exit_x
        )
        failure = collision or (out and not success)
        terminated = success or failure
        truncated = self.steps >= cfg.max_steps and not terminated
        reason = (
            "success"
            if success
            else "collision"
            if collision
            else "out"
            if failure
            else "timeout"
            if truncated
            else "running"
        )
        reward = 2.0 * (self._potential(self.world) - self._potential(previous)) - 0.005
        reward -= 0.001 * (command.vx**2 + command.vy**2 + command.omega**2)
        reward += 10.0 if success else -5.0 if failure else 0.0
        self.finished = terminated or truncated
        return (
            observe(self.world, self.key, self.intent, cfg),
            float(reward),
            bool(terminated),
            bool(truncated),
            self._info(success, reason),
        )

    def close(self):
        self.backend.close()
