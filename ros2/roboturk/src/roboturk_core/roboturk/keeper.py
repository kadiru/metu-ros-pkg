"""Bounded reactive keeper and goal-conditioned draw/finish micro behavior.

Opponent and phase/aim selection are analytic. The checkpoint alone produces
attacker motion and kick timing. All progress is measured to the fixed goal center.
"""

from dataclasses import replace
from collections import deque
import json
import math
from pathlib import Path
import gymnasium as gym
import numpy as np
from .model import (
    compatible_policy_config,
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
    wrap,
)
from .defended import DefendedEnv
from .shoot import ShootEnv
from .shoot_contract import shoot_observation, shoot_command, shoot_action, SHOT_SPEED
from .capture import CaptureTeacher

KEEPER = RobotKey("yellow", 1)
KEEPER_CONTRACT = "roboturk.keeper_finish.v1"


class ReactiveKeeper:
    """Delayed, acceleration-limited lateral keeper; two fixed reaction styles."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.configure(np.random.default_rng(0))

    def configure(self, rng):
        self.speed = float(rng.uniform(0.65, 0.95))
        self.acceleration = float(rng.uniform(2.0, 3.0))
        self.delay = int(rng.integers(4, 8))
        self.style = str(rng.choice(["aim_tracking", "ball_line"]))
        self.initial_y = float(rng.uniform(-0.15, 0.15))
        self.targets = deque([self.initial_y] * self.delay)
        self.velocity = 0.0

    def command(self, w):
        r, b, cfg = w.robot(KEEPER), w.ball, self.cfg
        sign = w.attack_sign
        anchor = sign * (cfg.length / 2 - 0.18)
        goal = sign * cfg.length / 2
        target = b.y * sign * (goal - anchor) / max(0.01, sign * (goal - b.x))
        holder = next(
            (p for p in w.robots if p.key.team == "blue" and possession(w, p.key, cfg)), None
        )
        if self.style == "aim_tracking" and holder and sign * (goal - b.x) < 2.5:
            dx = math.cos(holder.yaw)
            if sign * dx > 0.1:
                target = b.y + (anchor - b.x) * math.sin(holder.yaw) / dx
        if sign * b.vx > 0.3:
            time = (anchor - b.x) / b.vx
            if 0 < time < 2.0:
                target = b.y + b.vy * time
        margin = cfg.goal_width / 2 - cfg.robot_radius
        self.targets.append(float(np.clip(target, -margin, margin)))
        delayed = self.targets.popleft()
        desired = float(np.clip(8 * (delayed - r.y), -self.speed, self.speed))
        self.velocity += float(
            np.clip(
                desired - self.velocity, -self.acceleration * cfg.dt, self.acceleration * cfg.dt
            )
        )
        vx, vy = body_vector(4 * (anchor - r.x), self.velocity, r.yaw)
        angle = wrap(math.atan2(b.y - r.y, b.x - r.x) - r.yaw)
        return bounded(Command(vx, vy, float(np.clip(5 * angle, -4, 4)), 1), cfg)


class KeeperBackend:
    def __init__(self, base, cfg):
        self.base, self.cfg = base, cfg
        self.keeper = ReactiveKeeper(cfg)
        self.name = base.name + "+reactive_keeper_v1"
        self.reset_metrics()

    def __getattr__(self, name):
        return getattr(self.base, name)

    @property
    def world(self):
        return self.base.world

    @world.setter
    def world(self, value):
        self.base.world = value

    @property
    def failure(self):
        return self.keeper_failure or getattr(self.base, "failure", None)

    def reset_metrics(self):
        self.keeper_failure = None
        self.contacts = self.collisions = self.checks = 0
        self.minimum_distance = 99.0
        self.distance_moved = 0.0

    def reset(self, world):
        self.reset_metrics()
        k = Robot(
            KEEPER,
            world.attack_sign * (self.cfg.length / 2 - 0.18),
            self.keeper.initial_y,
            math.pi if world.attack_sign == 1 else 0.0,
        )
        world = replace(world, robots=tuple(r for r in world.robots if r.key != KEEPER) + (k,))
        return self.base.reset(world)

    def step(self, commands):
        before = self.world.robot(KEEPER)
        result = self.base.step({**commands, KEEPER: self.keeper.command(self.world)})
        for w in getattr(self.base, "last_substep_worlds", [result]):
            k, b = w.robot(KEEPER), w.ball
            self.distance_moved += math.hypot(k.x - before.x, k.y - before.y)
            before = k
            distance = math.hypot(k.x - b.x, k.y - b.y)
            self.minimum_distance = min(self.minimum_distance, distance)
            self.checks += 1
            if distance <= self.cfg.robot_radius + self.cfg.ball_radius + 0.002 or possession(
                w, KEEPER, self.cfg
            ):
                self.contacts += 1
                self.keeper_failure = "keeper_save"
            for r in w.robots:
                if (
                    r.key.team == "blue"
                    and math.hypot(k.x - r.x, k.y - r.y) < 2 * self.cfg.robot_radius + 0.01
                ):
                    self.collisions += 1
                    self.keeper_failure = "keeper_collision"
        return result

    def evidence(self):
        return {
            **(self.base.evidence() if hasattr(self.base, "evidence") else {}),
            "keeper": "reactive_keeper_v1",
            "keeper_style": self.keeper.style,
            "keeper_speed_limit": self.keeper.speed,
            "keeper_acceleration": self.keeper.acceleration,
            "keeper_reaction_seconds": self.keeper.delay * self.cfg.dt,
            "keeper_ball_contacts": self.contacts,
            "keeper_robot_contacts": self.collisions,
            "keeper_physics_checks": self.checks,
            "minimum_ball_keeper_distance": self.minimum_distance,
            "keeper_distance_moved": self.distance_moved,
        }


class FinishPlan:
    """Scripted draw/finish tactic; latches phase, never overrides actor outputs."""

    def __init__(self, w, key, cfg):
        self.side = 1.0 if w.ball.y >= 0 else -1.0
        self.phase = 0
        self.arrival_time = None
        self.last_time = None
        self.target = (w.direction(key.team) * cfg.length / 2, self.side * 0.43)

    def parking(self, w, key, cfg):
        return (w.direction(key.team) * (cfg.length / 2 - 1.0), self.side * 0.18)

    def update(self, w, key, cfg):
        if self.last_time == w.time:
            return
        self.last_time = w.time
        px, py = self.parking(w, key, cfg)
        r, b, k = w.robot(key), w.ball, w.robot(KEEPER)
        angle = wrap(math.atan2(self.target[1] - b.y, self.target[0] - b.x) - r.yaw)
        ready = (
            math.hypot(px - b.x, py - b.y) < 0.09
            and abs(angle) < 0.05
            and math.hypot(r.vx, r.vy) < 0.15
        )
        if ready and self.arrival_time is None:
            self.arrival_time = w.time
        if (
            self.phase == 0
            and ready
            and (
                self.side * k.y > 0.22
                or (self.arrival_time is not None and w.time - self.arrival_time > 1.2)
            )
        ):
            self.phase = 1
            self.target = (self.target[0], -self.side * 0.43)


def finish_observation(w, key, plan, cfg):
    plan.update(w, key, cfg)
    r, b, k = w.robot(key), w.ball, w.robot(KEEPER)
    intent = Intent(Skill.SHOOT, (w.direction(key.team) * cfg.length / 2, 0))
    gx, gy = body_vector(plan.target[0] - b.x, plan.target[1] - b.y, r.yaw)
    px, py = plan.parking(w, key, cfg)
    dx, dy = body_vector(px - b.x, py - b.y, r.yaw)
    extra = [
        *(v / 3 for v in body_vector(k.x - r.x, k.y - r.y, r.yaw)),
        *body_vector(k.vx, k.vy, r.yaw),
        gx / 3,
        gy / 3,
        math.atan2(gy, gx),
        dx / 3,
        dy / 3,
        math.hypot(dx, dy) / 3,
        float(plan.phase),
        plan.side * w.attack_sign,
        w.attack_sign * k.y,
        w.attack_sign * k.vy,
        w.attack_sign * b.y,
    ]
    result = np.concatenate([shoot_observation(w, key, intent, cfg), extra]).astype(np.float32)
    if not np.all(np.isfinite(result)):
        raise ValueError("Nonfinite keeper observation")
    return result


class FinishTeacher:
    def reset(self):
        self.plan = None

    def __init__(self):
        self.reset()

    def command(self, w, key, intent, cfg):
        if self.plan is None:
            self.plan = FinishPlan(w, key, cfg)
        return self.planned_command(w, key, self.plan, cfg)

    def action(self, env):
        return shoot_action(self.planned_command(env.world, env.key, env.plan, env.cfg), env.cfg)

    def planned_command(self, w, key, plan, cfg):
        plan.update(w, key, cfg)
        r, b = w.robot(key), w.ball
        if math.hypot(b.vx, b.vy) > 1.5 and abs(b.x) > cfg.length / 2 - 1.8:
            return Command(kick=SHOT_SPEED)
        if not possession(w, key, cfg):
            return CaptureTeacher().command(w, key, Intent(Skill.SHOOT, plan.target), cfg)
        bx, by = body_vector(b.x - r.x, b.y - r.y, r.yaw)
        gx, gy = body_vector(plan.target[0] - b.x, plan.target[1] - b.y, r.yaw)
        angle = math.atan2(gy, gx)
        px, py = plan.parking(w, key, cfg)
        dx, dy = body_vector(px - b.x, py - b.y, r.yaw)
        if plan.phase and abs(angle) < 0.015 and math.hypot(r.vx, r.vy) < 0.22:
            return Command(kick=SHOT_SPEED)
        omega = float(
            np.clip(
                (16 if plan.phase else 2) * angle,
                -3.0 if plan.phase else -0.9,
                3.0 if plan.phase else 0.9,
            )
        )
        distance = max(1e-6, math.hypot(dx, dy))
        speed = 0 if plan.phase else min(0.65, 1.8 * distance) * max(0, math.cos(angle)) ** 2
        return bounded(
            Command(
                speed * dx / distance + 3 * (bx - cfg.possession_x),
                speed * dy / distance - cfg.possession_x * omega + 3 * by,
                omega,
                1,
            ),
            cfg,
        )


class KeeperFinishEnv(ShootEnv):
    contract = KEEPER_CONTRACT
    task_version = "keeper_finish.randomized.v1"

    def __init__(self, cfg=Config(), backend=None):
        super().__init__(cfg, backend)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (32,), dtype=np.float32)
        self.plan = None

    def begin_finish(self):
        self.plan = FinishPlan(self.world, self.key, self.cfg)

    def reset(self, *, seed=None, options=None):
        gym.Env.reset(self, seed=seed)
        rng = self.np_random
        keeper_seed = (
            seed + 17000000 if seed is not None else int(self.np_random.integers(0, 2**31))
        )
        self.backend.keeper.configure(np.random.default_rng(keeper_seed))
        sign = int(rng.choice([-1, 1]))
        self.key = RobotKey("blue", int(rng.integers(0, 2)))
        x, y = (
            sign * float(rng.uniform(-0.4, 1.1)),
            float(rng.choice([-1, 1]) * rng.uniform(1.5, 2.5)),
        )
        yaw = float(rng.uniform(-math.pi, math.pi))
        initial = World(
            0.0,
            (Robot(self.key, x, y, yaw),),
            Ball(
                x + self.cfg.possession_x * math.cos(yaw), y + self.cfg.possession_x * math.sin(yaw)
            ),
            Referee.RUNNING,
            sign,
        )
        self.world = self.backend.reset(initial)
        self.clear_metrics()
        self.intent = Intent(Skill.SHOOT, (sign * self.cfg.length / 2, 0.0))
        self.steps, self.dwell, self.finished = 0, 20, False
        self.acquired, self.capture_step = True, 0
        self.capture_distance = self.initial_target_distance = self.target_distance(self.world)
        self.scenario = {"attack_sign": sign, "moving": False}
        self.begin_finish()
        return self.observation(), self.info(False, "reset")

    def observation(self):
        return finish_observation(self.world, self.key, self.plan, self.cfg)

    def info(self, success, reason):
        return {
            **super().info(success, reason),
            **self.backend.evidence(),
            "finish_phase": self.plan.phase if self.plan else 0,
            "ball_final": [self.world.ball.x, self.world.ball.y],
            "keeper_final_y": self.world.robot(KEEPER).y,
            "aim_target": list(self.plan.target) if self.plan else None,
        }

    def step(self, action):
        result = super().step(action)
        if self.backend.failure:
            self.finished = True
            return result[0], -25.0, True, False, self.info(False, self.backend.failure)
        return result


class KeeperPlayEnv(DefendedEnv):
    task_version = "defender_keeper_pass_receive_score.randomized.v1"
    scoring_env_type = KeeperFinishEnv

    def __init__(self, cfg=Config(), backend=None):
        super().__init__(cfg, backend)
        self.backend = KeeperBackend(self.backend, cfg)

    def reset(self, *, seed=None, options=None):
        gym.Env.reset(self, seed=seed)
        keeper_seed = (
            seed + 17000000 if seed is not None else int(self.np_random.integers(0, 2**31))
        )
        self.backend.keeper.configure(np.random.default_rng(keeper_seed))
        return super().reset(seed=seed, options=options)


class FinishCheckpoint:
    def __init__(self, path, cfg=Config(), model=None):
        if model is None:
            from stable_baselines3 import PPO

            manifest = json.loads(Path(path).with_suffix(".json").read_text())
            if manifest["contract"] != KEEPER_CONTRACT or not compatible_policy_config(
                manifest["config"], cfg
            ):
                raise ValueError("Keeper finish manifest mismatch")
            model = PPO.load(path, device="cpu")
        if model.observation_space.shape != (32,) or model.action_space.shape != (4,):
            raise ValueError("Keeper finish checkpoint shape mismatch")
        self.model = model
        self.reset()

    def reset(self):
        self.plan = None

    def action(self, env):
        return self.model.predict(env.observation(), deterministic=True)[0]

    def command(self, w, key, intent, cfg):
        if self.plan is None:
            self.plan = FinishPlan(w, key, cfg)
        return shoot_command(
            self.model.predict(finish_observation(w, key, self.plan, cfg), deterministic=True)[0],
            cfg,
        )
