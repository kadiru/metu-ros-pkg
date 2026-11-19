"""Mobile learned opponents and versioned self-play observation contracts.

No line/goal-area position clamp or analytic movement controller is used. The
opponents have the entire field available, subject to actuator/referee limits.
Tactical receiving/finishing targets remain explicit scripted attacking context.
"""

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import gymnasium as gym
import numpy as np
from .model import Config, RobotKey, body_vector
from .defended import (
    DEFENDER,
    DEFENDED_CONTRACT,
    DefendedCheckpoint,
    defended_observation,
    tactical_state,
)
from .keeper import KEEPER, finish_observation
from .learned_opponents import (
    OpponentController,
    opponent_observation,
    LearnedDefendedBackend,
    LearnedKeeperBackend,
    LearnedOpponentPlayEnv,
)
from .adversarial_finish import PressureFinishPlan, PressureFinishEnv, PressureFinishCheckpoint
from .shoot_contract import shoot_command

CONTRACTS = {
    "defender": "roboturk.mobile_opponent.v1",
    "keeper": "roboturk.mobile_opponent.v1",
    "transfer": "roboturk.mobile_transfer.v1",
    "finish": "roboturk.mobile_finish.v1",
}
SHAPES = {"defender": (45, 3), "keeper": (45, 3), "transfer": (70, 7), "finish": (36, 4)}
ROLES = tuple(SHAPES)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_model(path, role, cfg):
    from stable_baselines3 import PPO

    meta = json.loads(Path(path).with_suffix(".json").read_text())
    if meta["contract"] != CONTRACTS[role] or meta["role"] != role or meta["config"] != asdict(cfg):
        raise ValueError("Self-play manifest mismatch: " + role)
    model = PPO.load(path, device="cpu")
    check_shape(model, role)
    return model


def check_shape(model, role):
    obs, actions = SHAPES[role]
    if model.observation_space.shape != (obs,) or model.action_space.shape != (actions,):
        raise ValueError("Self-play shape mismatch: " + role)


def relative_robot(w, observer, target, cfg):
    r, other = w.robot(observer), w.robot(target)
    return [
        *(v / 3 for v in body_vector(other.x - r.x, other.y - r.y, r.yaw)),
        *(v / cfg.max_speed for v in body_vector(other.vx, other.vy, r.yaw)),
    ]


def mobile_observation(w, c):
    r, sign = w.robot(c.key), w.attack_sign
    other = w.robot(DEFENDER if c.role == "keeper" else KEEPER)
    extra = [sign * r.x / 4.5, sign * r.y / 3]
    for i in range(2):
        extra.extend(relative_robot(w, c.key, RobotKey("blue", i), c.cfg))
    extra.extend([sign * (other.x - r.x) / 3, sign * (other.y - r.y) / 3])
    return np.concatenate([opponent_observation(w, c), extra]).astype(np.float32)


class MobileOpponent(OpponentController):
    def __init__(self, role, cfg=Config(), *, path=None, model=None):
        if role not in ("defender", "keeper"):
            raise ValueError("Unknown opponent role")
        self.role, self.cfg = role, cfg
        self.key = KEEPER if role == "keeper" else DEFENDER
        self.model = load_model(path, role, cfg) if model is None else model
        check_shape(self.model, role)
        self.sha256 = digest(path) if path is not None else None
        self.anchor_x = self.anchor_y = self.phase = 0.0
        self.period = 3.0
        self.configure(np.random.default_rng(0))

    def configure(self, rng):
        super().configure(rng)
        self.speed = float(
            rng.uniform(0.8, 1.15) if self.role == "keeper" else rng.uniform(0.55, 0.85)
        )
        self.acceleration = float(rng.uniform(2.0, 3.5))
        self.delay = int(rng.integers(2, 6)) if self.role == "keeper" else 0
        self.style = "mobile"
        self.reset_control()

    def action(self, world):
        return self.model.predict(mobile_observation(world, self), deterministic=True)[0]


class Excursion:
    """Measure freedom actually exercised, including every physics sample."""

    def reset_excursion(self, world, key):
        r = world.robot(key)
        self.origin = (r.x, r.y)
        self.extent = [r.x, r.x, r.y, r.y]
        self.outside_checks = 0

    def measure(self, world, key):
        r = world.robot(key)
        self.extent = [
            min(self.extent[0], r.x),
            max(self.extent[1], r.x),
            min(self.extent[2], r.y),
            max(self.extent[3], r.y),
        ]
        self.outside_checks += int(
            abs(r.x) > self.cfg.length / 2 - self.cfg.robot_radius
            or abs(r.y) > self.cfg.width / 2 - self.cfg.robot_radius
        )

    def excursion(self, role):
        return {
            role + "_x_excursion": max(abs(v - self.origin[0]) for v in self.extent[:2]),
            role + "_y_excursion": max(abs(v - self.origin[1]) for v in self.extent[2:]),
            role + "_outside_checks": self.outside_checks,
        }


class MobileDefendedBackend(LearnedDefendedBackend, Excursion):
    def reset(self, world):
        result = super().reset(world)
        self.reset_excursion(result, DEFENDER)
        return result

    def step(self, commands):
        result = super().step(commands)
        for w in getattr(self.base, "last_substep_worlds", [result]):
            self.measure(w, DEFENDER)
        return result

    def evidence(self):
        return {
            **super().evidence(),
            "opponent": "mobile_learned_defender_v1",
            **self.excursion("defender"),
        }


class MobileKeeperBackend(LearnedKeeperBackend, Excursion):
    def reset(self, world):
        result = super().reset(world)
        self.reset_excursion(result, KEEPER)
        return result

    def step(self, commands):
        result = super().step(commands)
        for w in getattr(self.base, "last_substep_worlds", [result]):
            self.measure(w, KEEPER)
        return result

    def evidence(self):
        return {
            **super().evidence(),
            "keeper": "mobile_learned_keeper_v1",
            **self.excursion("keeper"),
        }


def transfer_observation(w, passer, receiver, state, cfg):
    return np.concatenate(
        [
            defended_observation(w, passer, receiver, state, cfg),
            relative_robot(w, passer, KEEPER, cfg),
            relative_robot(w, receiver, KEEPER, cfg),
        ]
    ).astype(np.float32)


def finishing_observation(w, key, plan, cfg):
    return np.concatenate(
        [finish_observation(w, key, plan, cfg), relative_robot(w, key, DEFENDER, cfg)]
    ).astype(np.float32)


class MobileFinishPlan(PressureFinishPlan):
    def parking(self, w, key, cfg):
        # Scripted shot setup stays farther from a keeper able to leave its line.
        return (w.direction(key.team) * (cfg.length / 2 - 1.35), 0.0)


class MobileFinishEnv(PressureFinishEnv):
    contract = CONTRACTS["finish"]
    task_version = "mobile_self_play_finish.v1"

    def __init__(self, cfg=Config(), backend=None):
        super().__init__(cfg, backend)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (36,), dtype=np.float32)

    def begin_finish(self):
        self.plan = MobileFinishPlan(self.world, self.key, self.cfg)

    def observation(self):
        return finishing_observation(self.world, self.key, self.plan, self.cfg)


class MobilePlayEnv(LearnedOpponentPlayEnv):
    task_version = "mobile_self_play_transfer.v1"
    scoring_env_type = MobileFinishEnv

    def __init__(self, cfg, backend, defender, keeper):
        super().__init__(cfg, backend, defender, keeper)
        self.backend = MobileKeeperBackend(
            MobileDefendedBackend(backend, cfg, defender), cfg, keeper
        )
        self.backend.name = backend.name + "+mobile_self_play_v1"
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (70,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.backend.defender.configure(self.np_random)
        return self.observation(), self.info(False, "reset")

    def observation(self):
        return transfer_observation(self.world, self.passer, self.receiver, self.state, self.cfg)


class MobileTransfer(DefendedCheckpoint):
    def __init__(self, path=None, cfg=Config(), model=None):
        self.model = load_model(path, "transfer", cfg) if model is None else model
        check_shape(self.model, "transfer")
        self.contract = DEFENDED_CONTRACT  # Shared command decoder and tactical state.

    def initial_state(self, world, passer, receiver, cfg):
        return tactical_state(world, passer, receiver, cfg)

    def predict(self, world, passer, receiver, state, cfg):
        return self.model.predict(
            transfer_observation(world, passer, receiver, state, cfg), deterministic=True
        )[0]


class MobileFinish(PressureFinishCheckpoint):
    def __init__(self, path=None, cfg=Config(), model=None):
        self.model = load_model(path, "finish", cfg) if model is None else model
        check_shape(self.model, "finish")
        self.reset()

    def command(self, w, key, intent, cfg):
        if self.plan is None:
            self.plan = MobileFinishPlan(w, key, cfg)
        action = self.model.predict(
            finishing_observation(w, key, self.plan, cfg), deterministic=True
        )[0]
        return shoot_command(action, cfg)


def make_play(backend, cfg, models):
    defender = MobileOpponent("defender", cfg, model=models["defender"])
    keeper = MobileOpponent("keeper", cfg, model=models["keeper"])
    return (
        MobilePlayEnv(cfg, backend, defender, keeper),
        MobileTransfer(cfg=cfg, model=models["transfer"]),
        MobileFinish(cfg=cfg, model=models["finish"]),
    )


def outcomes(result):
    from collections import Counter

    counts = Counter(r["reason"] for r in result["episodes"])
    clean = counts["goal"] + counts["defender_interception"] + counts["keeper_save"]
    return {
        "goals": counts["goal"],
        "defensive_stops": counts["defender_interception"] + counts["keeper_save"],
        "clean_outcomes": clean,
        "collisions": sum(v for k, v in counts.items() if "collision" in k),
        "reasons": dict(counts),
    }
