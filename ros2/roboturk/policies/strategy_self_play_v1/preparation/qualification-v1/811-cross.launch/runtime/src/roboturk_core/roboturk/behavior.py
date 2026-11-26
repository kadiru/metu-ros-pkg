from dataclasses import asdict
import json
import math
from pathlib import Path
from typing import Protocol

from .contract import CONTRACT_VERSION, decode_action, observe
from .model import (
    Command,
    Config,
    Intent,
    RobotKey,
    Skill,
    World,
    body_vector,
    bounded,
    constrain_skill,
    possession,
    wrap,
)


class Policy(Protocol):
    def command(self, world: World, key: RobotKey, intent: Intent, cfg: Config) -> Command: ...


class AnalyticPolicy:
    """Small baseline controller, without obstacle avoidance or competition guarantees."""

    def command(self, world, key, intent, cfg):
        r, ball = world.robot(key), world.ball
        if intent.skill == Skill.HOLD or (intent.skill != Skill.MOVE and not ball.visible):
            return Command()
        target = intent.target
        dribbler = kick = 0.0
        facing = math.atan2(target[1] - r.y, target[0] - r.x)
        has_ball = possession(world, key, cfg)
        if intent.skill in (Skill.CHASE, Skill.DRIBBLE, Skill.PASS, Skill.SHOOT):
            facing = math.atan2(intent.target[1] - ball.y, intent.target[0] - ball.x)
            dribbler = 1.0
            if not has_ball or intent.skill == Skill.CHASE:
                target = (
                    ball.x - cfg.possession_x * math.cos(facing),
                    ball.y - cfg.possession_x * math.sin(facing),
                )
            elif intent.skill in (Skill.PASS, Skill.SHOOT):
                target = (r.x, r.y)
                tolerance = 0.025 if intent.skill == Skill.PASS else 0.08
                if abs(wrap(facing - r.yaw)) < tolerance:
                    distance = math.hypot(intent.target[0] - ball.x, intent.target[1] - ball.y)
                    kick = min(cfg.max_kick, max(1.2, 1.5 * math.sqrt(distance)))
                    dribbler = 0.0
        elif intent.skill == Skill.RECEIVE:
            facing = math.atan2(ball.y - r.y, ball.x - r.x)
            # Intercept nearby rolling balls; remain at the assigned receiving point otherwise.
            if math.hypot(ball.x - r.x, ball.y - r.y) < 0.6:
                target = (
                    ball.x - cfg.possession_x * math.cos(facing),
                    ball.y - cfg.possession_x * math.sin(facing),
                )
            dribbler = 1.0
            if has_ball:
                target = (r.x, r.y)
        dx, dy = body_vector(target[0] - r.x, target[1] - r.y, r.yaw)
        speed_limit = 0.65 if has_ball else cfg.max_speed
        speed = max(1.0, 2.5 * math.hypot(dx, dy) / speed_limit)
        return bounded(
            Command(2.5 * dx / speed, 2.5 * dy / speed, 5.0 * wrap(facing - r.yaw), dribbler, kick),
            cfg,
        )


class CheckpointPolicy:
    """SB3 inference with an explicit contract/config/skill manifest."""

    def __init__(self, path: str, skill: Skill, cfg: Config):
        from stable_baselines3 import PPO

        manifest = json.loads(Path(path).with_suffix(".json").read_text())
        from .capture_contract import CAPTURE_CONTRACT
        from .dribble_contract import DRIBBLE_CONTRACT
        from .shoot_contract import SHOOT_CONTRACT, SHOOT_CONTRACT_V1

        self.contract = manifest["contract"]
        if (
            self.contract
            not in (
                CONTRACT_VERSION,
                CAPTURE_CONTRACT,
                DRIBBLE_CONTRACT,
                SHOOT_CONTRACT,
                SHOOT_CONTRACT_V1,
            )
            or manifest["skill"] != skill.value
            or manifest["config"] != asdict(cfg)
            or (self.contract == CAPTURE_CONTRACT and skill != Skill.CHASE)
            or (self.contract == DRIBBLE_CONTRACT and skill != Skill.DRIBBLE)
            or (self.contract in (SHOOT_CONTRACT, SHOOT_CONTRACT_V1) and skill != Skill.SHOOT)
        ):
            raise ValueError("Policy manifest does not match the runtime contract/skill/config")
        self.skill = skill
        self.model = PPO.load(path, device="cpu")

    def command(self, world, key, intent, cfg):
        if intent.skill != self.skill:
            raise ValueError("Policy invoked for a different skill")
        from .capture_contract import CAPTURE_CONTRACT, capture_command, capture_observation
        from .dribble_contract import DRIBBLE_CONTRACT, dribble_observation
        from .shoot_contract import (
            SHOOT_CONTRACT,
            SHOOT_CONTRACT_V1,
            shoot_command,
            shoot_observation,
        )

        if self.contract in (SHOOT_CONTRACT, SHOOT_CONTRACT_V1):
            if not world.ball.visible or not world.robot(key).visible:
                return Command()
            observation = shoot_observation(world, key, intent, cfg)
            if self.contract == SHOOT_CONTRACT_V1:
                observation = observation[:16]
            action, _ = self.model.predict(observation, deterministic=True)
            return shoot_command(action, cfg)

        if self.contract in (CAPTURE_CONTRACT, DRIBBLE_CONTRACT):
            if not world.ball.visible or not world.robot(key).visible:
                return Command()
            observation = (
                capture_observation(world, key, cfg)
                if self.contract == CAPTURE_CONTRACT
                else dribble_observation(world, key, intent, cfg)
            )
            action, _ = self.model.predict(observation, deterministic=True)
            return capture_command(action, cfg)
        action, _ = self.model.predict(observe(world, key, intent, cfg), deterministic=True)
        return decode_action(action, cfg)


class BehaviorLibrary:
    def __init__(self):
        self.fallback = AnalyticPolicy()
        self.policies: dict[Skill, Policy] = {}

    def register(self, skill: Skill, policy: Policy):
        self.policies[skill] = policy

    def command(self, world: World, key: RobotKey, intent: Intent, cfg: Config) -> Command:
        command = self.policies.get(intent.skill, self.fallback).command(world, key, intent, cfg)
        return constrain_skill(command, intent.skill)
