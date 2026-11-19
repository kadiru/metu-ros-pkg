"""Warm-start attackers against frozen, hash-identified neural opponents."""

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import torch
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback
from .dribble_refinement import AnchoredPPO
from .defended import DefendedTeacher, DEFENDED_CONTRACT
from .keeper import FinishTeacher
from .adversarial_finish import PressureFinishEnv, PRESSURE_FINISH_CONTRACT
from .learned_opponents import LearnedOpponentPlayEnv, LearnedKeeperBackend
from .team import evaluate_team
from .team_training import ModelTeam, collect as team_collect, fit_actor, groups as team_groups
from .micro_training import Actor, collect, evaluate, fit, phase_groups


def train_attackers(
    backend, cfg, output, defender, keeper, transfer_path, finish_path, steps=65536, seed=137
):
    torch.set_num_threads(1)
    torch.manual_seed(seed)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    opponents = {"defender": defender.sha256, "keeper": keeper.sha256}
    for kind, source, base in (
        ("transfer", transfer_path, 1400000),
        ("finish", finish_path, 1500000),
    ):
        env = (
            LearnedOpponentPlayEnv(cfg, backend, defender, keeper)
            if kind == "transfer"
            else PressureFinishEnv(cfg, LearnedKeeperBackend(backend, cfg, keeper))
        )
        target = output / kind
        target.mkdir(exist_ok=True)
        model = AnchoredPPO.load(source, env=Monitor(env), device="cpu", learning_rate=3e-7)
        model.set_random_seed(seed)
        teacher = DefendedTeacher() if kind == "transfer" else FinishTeacher()
        metadata = {
            "contract": DEFENDED_CONTRACT if kind == "transfer" else PRESSURE_FINISH_CONTRACT,
            "config": asdict(cfg),
            "task": env.task_version,
            "backend": env.backend.name,
            "source_checkpoint_sha256": hashlib.sha256(Path(source).read_bytes()).hexdigest(),
            "frozen_opponents": opponents,
            "training_seed": seed,
            "warm_start_ppo_steps": model.num_timesteps,
            "method": "warm-start + native Webots demonstrations/DAgger + anchored PPO against frozen neural opponents",
            "demonstration_seed_range": [base, base + 299],
            "dagger_seed_range": [base + 20000, base + 20149],
            "development_seed_range": [base + 10000, base + 10059],
            "qualification_seed_range": [1650000, 1650199],
            "demo_seed_range": [1660000, 1660005],
            "anchor_mean_square_delta": 1e-6,
            "anchor_max_normalized_action_delta": 0.01,
            "validated_for_hardware": False,
        }

        def report(stage):
            result = (
                evaluate_team(env, ModelTeam(model), 60, base + 10000)
                if kind == "transfer"
                else evaluate(env, Actor(model), 60, base + 10000)
            )
            (target / (stage + "-development.json")).write_text(json.dumps(result, indent=2) + "\n")
            print(f"Attacker {kind} {stage}: {result['successes']}/60", flush=True)
            return result

        def save(stage):
            model.save(target / stage)
            (target / (stage + ".json")).write_text(
                json.dumps(
                    {
                        **metadata,
                        "stage": stage,
                        "actual_steps": model.num_timesteps,
                        "current_experiment_ppo_steps": model.num_timesteps
                        if stage == "policy"
                        else 0,
                    },
                    indent=2,
                )
                + "\n"
            )

        report("baseline")
        if kind == "transfer":
            x, y, records = team_collect(env, 300, base, teacher=teacher)
            fit_actor(model, x, y, 60, seed)
        else:
            x, y, records = collect(env, teacher, 300, base)
            fit(model, x, y, 60, seed, "keeper")
        (target / "teacher-collection.json").write_text(json.dumps(records, indent=2) + "\n")
        save("bc")
        results = {"bc": report("bc")}
        if kind == "transfer":
            dx, dy, records = team_collect(env, 150, base + 20000, model, teacher=teacher)
        else:
            dx, dy, records = collect(env, teacher, 150, base + 20000, model)
        x, y = np.concatenate([x, dx]), np.concatenate([y, dy])
        (target / "dagger-collection.json").write_text(json.dumps(records, indent=2) + "\n")
        np.savez_compressed(target / "demonstrations.npz", observations=x, actions=y)
        if kind == "transfer":
            fit_actor(model, x, y, 60, seed + 1)
        else:
            fit(model, x, y, 60, seed + 1, "keeper")
        save("dagger")
        results["dagger"] = report("dagger")
        best = max(
            results,
            key=lambda s: (
                results[s]["successes"],
                -results[s].get("possession_loss_episodes", 0),
                -(results[s]["mean_success_seconds"] or 1000),
            ),
        )
        metadata["anchor_reference"] = best
        model = AnchoredPPO.load(
            target / (best + ".zip"), env=Monitor(env), device="cpu", learning_rate=3e-7
        )
        model.set_random_seed(seed)
        indices = team_groups(x, y) if kind == "transfer" else phase_groups(x, y, "keeper")
        model.prepare_anchor(x, seed, [g for g in indices if len(g)])
        for group in model.anchor_optimizer.param_groups:
            group["lr"] = 3e-6
        started = time.monotonic()

        class Progress(BaseCallback):
            def _on_step(self):
                if self.num_timesteps % 8192 == 0:
                    print(
                        f"Attacker {kind} PPO {self.num_timesteps}/{steps}, {time.monotonic() - started:.1f}s",
                        flush=True,
                    )
                return True

        model.learn(total_timesteps=steps, callback=Progress())
        metadata.update(
            anchor_updates=model.anchor_updates, anchor_rejected_updates=model.anchor_rejections
        )
        save("policy")
        (target / "anchor-updates.json").write_text(json.dumps(model.anchor_log, indent=2) + "\n")
        report("policy")
