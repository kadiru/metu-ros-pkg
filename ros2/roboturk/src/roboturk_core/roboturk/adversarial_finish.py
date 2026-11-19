"""Closer, centered finish for the frozen learned-keeper curriculum."""

from dataclasses import asdict
import json
from pathlib import Path
from .keeper import FinishPlan, KeeperFinishEnv, FinishCheckpoint, finish_observation
from .shoot_contract import shoot_command
from .model import Config

PRESSURE_FINISH_CONTRACT = "roboturk.pressure_finish.v1"


class PressureFinishPlan(FinishPlan):
    """Scripted parking/aim context; the actor generates every motion and release."""

    def parking(self, w, key, cfg):
        return (w.direction(key.team) * (cfg.length / 2 - 0.70), 0.0)


class PressureFinishEnv(KeeperFinishEnv):
    contract = PRESSURE_FINISH_CONTRACT
    task_version = "learned_keeper_pressure_finish.randomized.v1"

    def begin_finish(self):
        self.plan = PressureFinishPlan(self.world, self.key, self.cfg)


class PressureFinishCheckpoint(FinishCheckpoint):
    def __init__(self, path, cfg=Config(), model=None):
        if model is None:
            from stable_baselines3 import PPO

            manifest = json.loads(Path(path).with_suffix(".json").read_text())
            if manifest["contract"] != PRESSURE_FINISH_CONTRACT or manifest["config"] != asdict(
                cfg
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
