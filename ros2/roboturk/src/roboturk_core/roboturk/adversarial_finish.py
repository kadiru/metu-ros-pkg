"""Closer, centered finish for the frozen learned-keeper curriculum."""

import json
import math
from pathlib import Path
from .keeper import FinishPlan, KeeperFinishEnv, FinishCheckpoint, finish_observation
from .shoot_contract import shoot_command
from .model import Config, possession, wrap, compatible_policy_config

PRESSURE_FINISH_CONTRACT = "roboturk.pressure_finish.v2"


class CloseFinishPlan(FinishPlan):
    """Scripted parking/aim context; the actor generates every motion and release."""

    def parking(self, w, key, cfg):
        return (w.direction(key.team) * (cfg.length / 2 - 0.70), 0.0)


def predicted_lane_clearance(w, target, cfg):
    """Perpendicular shot/keeper separation using a short constant-velocity forecast."""
    from .keeper import KEEPER

    b, k = w.ball, w.robot(KEEPER)
    dx, dy = target[0] - b.x, target[1] - b.y
    fraction = max(0.0, min(1.0, (k.x - b.x) / dx)) if abs(dx) > 1e-6 else 0.0
    flight = fraction * math.hypot(dx, dy) / 3.5
    future_y = k.y + k.vy * flight
    return abs(future_y - b.y - fraction * dy) * abs(dx) / max(1e-6, math.hypot(dx, dy))


class PressureFinishPlan(CloseFinishPlan):
    """Choose an open corner; target selection never supplies an actuator command."""

    arrival_radius = 0.07
    arrival_angle = 0.05

    def update(self, w, key, cfg):
        if self.last_time == w.time:
            return
        self.last_time = w.time
        if not possession(w, key, cfg):
            return
        r, b = w.robot(key), w.ball
        angle = wrap(math.atan2(self.target[1] - b.y, self.target[0] - b.x) - r.yaw)
        px, py = self.parking(w, key, cfg)
        ready = (
            math.hypot(px - b.x, py - b.y) < self.arrival_radius and math.hypot(r.vx, r.vy) < 0.15
        )
        if self.phase == 0 and ready and abs(angle) < self.arrival_angle:
            self.phase = 1
        if self.phase and abs(angle) < 0.12:
            other = (self.target[0], -self.target[1])
            current = predicted_lane_clearance(w, self.target, cfg)
            alternative = predicted_lane_clearance(w, other, cfg)
            if current < 0.20 and alternative > current + 0.03:
                self.target = other


class PressureFinishEnv(KeeperFinishEnv):
    contract = PRESSURE_FINISH_CONTRACT
    task_version = "learned_keeper_pressure_finish.randomized.v2"

    def begin_finish(self):
        self.plan = PressureFinishPlan(self.world, self.key, self.cfg)


class PressureFinishCheckpoint(FinishCheckpoint):
    def __init__(self, path, cfg=Config(), model=None):
        if model is None:
            from stable_baselines3 import PPO

            manifest = json.loads(Path(path).with_suffix(".json").read_text())
            if manifest["contract"] != PRESSURE_FINISH_CONTRACT or not compatible_policy_config(
                manifest["config"], cfg
            ):
                raise ValueError("Pressure finish manifest mismatch")
            model = PPO.load(path, device="cpu")
        super().__init__(path, cfg, model=model)

    def command(self, w, key, intent, cfg):
        if self.plan is None:
            self.plan = PressureFinishPlan(w, key, cfg)
        action = self.model.predict(finish_observation(w, key, self.plan, cfg), deterministic=True)[
            0
        ]
        return shoot_command(action, cfg)
