"""Neural opponent controls with shared, bounded actuator dynamics.

Geometric threat projections are observations, not commanded targets. Both
translations and yaw rate come from a checkpoint. Role anchors, physical limits,
reaction delay and dribbler enable are fixed context. Teachers live in training.
"""

from collections import deque
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from .model import Command, Config, Referee, body_vector, possession, wrap
from .defended import DEFENDER, DefendedBackend
from .keeper import KEEPER, KeeperBackend, KeeperPlayEnv, KeeperFinishEnv

CONTRACT = "roboturk.continuous_opponent.v1"
OBSERVATIONS = 33


def opponent_observation(w, controller):
    c, cfg = controller, controller.cfg
    r, b, sign = w.robot(c.key), w.ball, w.attack_sign
    anchor = sign * (cfg.length / 2 - 0.18) if c.role == "keeper" else c.anchor_x
    crossing_time = (anchor - b.x) / b.vx if abs(b.vx) > 0.15 else -1
    crossing = 0 <= crossing_time <= 2.5
    holder = next((p for p in w.robots if p.key.team == "blue" and possession(w, p.key, cfg)), None)
    aiming = holder is not None and sign * math.cos(holder.yaw) > 0.1
    aim_y = b.y + (anchor - b.x) * math.tan(holder.yaw) if aiming else 0.0
    phase = 2 * math.pi * w.time / c.period + c.phase
    out = np.array(
        [
            sign * (anchor - r.x),
            sign * (c.anchor_y - r.y),
            sign * r.y,
            sign * r.vx,
            sign * r.vy,
            r.omega / 4,
            sign * math.sin(r.yaw),
            sign * math.cos(r.yaw),
            sign * (b.x - r.x) / 3,
            sign * (b.y - r.y) / 3,
            sign * b.vx / 4,
            sign * b.vy / 4,
            wrap(math.atan2(b.y - r.y, b.x - r.x) - r.yaw) / math.pi,
            sign * b.x / (cfg.length / 2),
            sign * b.y / 3,
            c.speed,
            c.acceleration / 3,
            c.delay / 8,
            float(c.style == "aim_tracking"),
            sign * math.sin(phase),
            sign * math.cos(phase),
            float(crossing),
            sign * float(np.clip(b.y + b.vy * crossing_time, -3, 3)) / 3 if crossing else 0,
            float(holder is not None),
            float(aiming),
            sign * float(np.clip(aim_y, -3, 3)) / 3,
            sign * (sign * cfg.length / 2 - b.x) / (cfg.length / 2),
            holder.omega / 4 if holder else 0,
            float(possession(w, c.key, cfg)),
            float(c.role == "keeper"),
            *c.previous,
        ],
        np.float32,
    )
    if out.shape != (OBSERVATIONS,) or not np.isfinite(out).all():
        raise ValueError("Invalid opponent observation")
    return out


class OpponentController:
    """Checkpoint inference only; no analytic movement fallback."""

    def __init__(self, role, cfg=Config(), *, path=None, model=None):
        if role not in ("defender", "keeper"):
            raise ValueError("Unknown opponent role")
        self.role, self.cfg = role, cfg
        self.key = KEEPER if role == "keeper" else DEFENDER
        self.sha256 = None
        if model is None:
            from stable_baselines3 import PPO

            if path is None:
                raise ValueError("A learned opponent checkpoint is required")
            meta = json.loads(Path(path).with_suffix(".json").read_text())
            if (
                meta["contract"] != CONTRACT
                or meta["role"] != role
                or meta["config"] != asdict(cfg)
            ):
                raise ValueError("Opponent manifest mismatch")
            model = PPO.load(path, device="cpu")
            self.sha256 = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        if model.observation_space.shape != (OBSERVATIONS,) or model.action_space.shape != (3,):
            raise ValueError("Opponent shape mismatch")
        self.model = model
        self.anchor_x = self.anchor_y = self.phase = 0.0
        self.period = 3.0
        self.configure(np.random.default_rng(0))

    def configure(self, rng):
        self.speed = float(rng.uniform(0.65, 0.95)) if self.role == "keeper" else 0.4
        self.acceleration = float(rng.uniform(2, 3)) if self.role == "keeper" else 100.0
        self.delay = int(rng.integers(4, 8)) if self.role == "keeper" else 0
        self.style = (
            str(rng.choice(["aim_tracking", "ball_line"])) if self.role == "keeper" else "lane"
        )
        self.initial_y = float(rng.uniform(-0.15, 0.15))
        self.reset_control()

    def reset_control(self):
        self.previous = np.zeros(3, np.float32)
        self.queue = deque([np.zeros(3, np.float32) for _ in range(self.delay)])
        self.calls = 0
        self.last_time = None
        self.last_command = Command()

    def action(self, world):
        return self.model.predict(opponent_observation(world, self), deterministic=True)[0]

    def command(self, world):
        if (
            world.referee != Referee.RUNNING
            or not world.ball.visible
            or not world.robot(self.key).visible
        ):
            return Command()
        if self.last_time == world.time:
            return self.last_command
        action = np.asarray(self.action(world), dtype=np.float32)
        if action.shape != (3,) or not np.isfinite(action).all():
            raise ValueError("Invalid learned opponent action")
        a = np.clip(action, -1, 1)
        self.queue.append(a)
        a = self.queue.popleft().copy()
        a[:2] /= max(1.0, float(np.linalg.norm(a[:2])))
        desired = a[:2] * self.speed
        old = self.previous[:2] * self.speed
        delta = desired - old
        delta /= max(1.0, float(np.linalg.norm(delta)) / (self.acceleration * self.cfg.dt))
        velocity = old + delta
        self.previous = np.array([*velocity / self.speed, a[2]], np.float32)
        r = world.robot(self.key)
        bx, by = body_vector(
            world.attack_sign * float(velocity[0]), world.attack_sign * float(velocity[1]), r.yaw
        )
        self.last_command = Command(bx, by, float(a[2]) * 4, 1)
        self.last_time = world.time
        self.calls += 1
        return self.last_command


class LearnedDefendedBackend(DefendedBackend):
    def __init__(self, base, cfg, controller):
        super().__init__(base, cfg)
        self.defender = controller
        self.name = base.name + "+learned_lane_v1"

    def reset(self, world):
        self.defender.reset_control()
        return super().reset(world)

    def evidence(self):
        return {
            **super().evidence(),
            "opponent": "learned_lane_v1",
            "defender_checkpoint_sha256": self.defender.sha256,
            "defender_policy_calls": self.defender.calls,
        }


class LearnedKeeperBackend(KeeperBackend):
    def __init__(self, base, cfg, controller):
        super().__init__(base, cfg)
        self.keeper = controller
        self.name = base.name + "+learned_keeper_v1"

    def reset(self, world):
        self.keeper.reset_control()
        return super().reset(world)

    def evidence(self):
        return {
            **super().evidence(),
            "keeper": "learned_keeper_v1",
            "keeper_checkpoint_sha256": self.keeper.sha256,
            "keeper_policy_calls": self.keeper.calls,
        }


class LearnedOpponentPlayEnv(KeeperPlayEnv):
    task_version = "learned_opponents_pass_receive_score.randomized.v1"

    def __init__(self, cfg, backend, defender, keeper):
        super().__init__(cfg, backend)
        self.backend = LearnedKeeperBackend(
            LearnedDefendedBackend(backend, cfg, defender), cfg, keeper
        )


def learned_finish_env(cfg, backend, keeper):
    env = KeeperFinishEnv(cfg, LearnedKeeperBackend(backend, cfg, keeper))
    env.task_version = "learned_keeper_finish.randomized.v1"
    return env
