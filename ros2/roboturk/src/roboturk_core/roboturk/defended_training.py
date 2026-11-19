"""Defender-aware transfer training with a frozen independent lane opponent."""

import hashlib
import json
import os
from pathlib import Path
import time
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback
from .dribble_refinement import AnchoredPPO
from .team import evaluate_team
from .team_training import ModelTeam, collect, fit_actor, groups, save
from .defended import DEFENDED_CONTRACT, DefendedTeacher


def train_defended(env, output, steps=65536, seed=67):
    torch.set_num_threads(1)
    torch.manual_seed(seed)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    demos = int(os.environ.get("ROBOTURK_DEMOS", "300"))
    dagger = int(os.environ.get("ROBOTURK_DAGGER", "150"))
    model = AnchoredPPO(
        "MlpPolicy",
        Monitor(env),
        seed=seed,
        device="cpu",
        n_steps=1024,
        batch_size=256,
        n_epochs=5,
        learning_rate=3e-7,
        clip_range=0.1,
        ent_coef=0.0,
        gamma=0.995,
        verbose=0,
        policy_kwargs={"net_arch": dict(pi=[192, 192], vf=[192, 192]), "log_std_init": -4.0},
    )
    checkpoint = Path(os.environ["ROBOTURK_CHECKPOINT"])
    previous = PPO.load(checkpoint, device="cpu").policy.state_dict()
    current = model.policy.state_dict()
    for name, value in current.items():
        old = previous[name]
        if old.shape == value.shape:
            current[name] = old
        elif name.endswith("policy_net.0.weight") or name.endswith("value_net.0.weight"):
            value.zero_()
            value[:, :36] = old
        else:
            raise ValueError(f"Unsupported warm-start shape: {name}")
    # Receiver outputs changed from body velocities to the attack frame.
    current["action_net.weight"][4:6].zero_()
    current["action_net.bias"][4:6].zero_()
    model.policy.load_state_dict(current)
    metadata = {
        "contract": DEFENDED_CONTRACT,
        "source_checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
        "opponent": "reactive_lane_defender_v1",
        "defender_speed_range": [0.25, 0.55],
        "seed": seed,
        "demonstration_seed_range": [600000, 600000 + demos - 1],
        "dagger_seed_range": [620000, 620000 + dagger - 1],
        "development_seed_range": [610000, 610059],
        "qualification_seed_range": [650000, 650199],
        "demo_seed_range": [660000, 660005],
        "learning_rate": 3e-7,
        "anchor_max_normalized_action_delta": 0.01,
        "anchor_mean_square_delta": 1e-6,
    }
    x, y, records = collect(env, demos, 600000, teacher=DefendedTeacher())
    (output / "teacher-collection.json").write_text(json.dumps(records, indent=2) + "\n")
    fit_actor(model, x, y, 100, seed)
    save(model, output / "bc", env, metadata, "cloned")
    bc = evaluate_team(env, ModelTeam(model), 60, 610000)
    (output / "bc-development.json").write_text(json.dumps(bc, indent=2) + "\n")
    print(f"Cloned team development: {bc['successes']}/60", flush=True)
    best = "bc"
    best_result = bc
    if dagger:
        dx, dy, records = collect(env, dagger, 620000, model, teacher=DefendedTeacher())
        (output / "dagger-collection.json").write_text(json.dumps(records, indent=2) + "\n")
        x, y = np.concatenate([x, dx]), np.concatenate([y, dy])
        fit_actor(model, x, y, 60, seed + 1)
        save(model, output / "dagger", env, metadata, "dagger")
        result = evaluate_team(env, ModelTeam(model), 60, 610000)
        (output / "dagger-development.json").write_text(json.dumps(result, indent=2) + "\n")
        print(f"DAgger team development: {result['successes']}/60", flush=True)
        if (result["successes"], -(result["mean_success_seconds"] or 1000)) > (
            bc["successes"],
            -(bc["mean_success_seconds"] or 1000),
        ):
            best, best_result = "dagger", result
    np.savez_compressed(output / "demonstrations.npz", observations=x, actions=y)
    metadata["anchor_reference"] = best
    print(
        f"Selected {best} on development ({best_result['successes']}/60), reserved qualification untouched",
        flush=True,
    )
    model = AnchoredPPO.load(
        output / (best + ".zip"), env=Monitor(env), device="cpu", learning_rate=3e-7
    )
    model.prepare_anchor(x, seed, [g for g in groups(x, y) if len(g)])
    for group in model.anchor_optimizer.param_groups:
        group["lr"] = 3e-6
    metadata["anchor_projection_learning_rate"] = 3e-6
    start = time.monotonic()

    class Progress(BaseCallback):
        def _on_step(self):
            if self.num_timesteps % 8192 == 0:
                print(
                    f"Team anchored PPO {self.num_timesteps}/{steps}; {time.monotonic() - start:.1f}s",
                    flush=True,
                )
            return True

    model.learn(total_timesteps=steps, callback=Progress())
    metadata.update(
        anchor_updates=model.anchor_updates, anchor_rejected_updates=model.anchor_rejections
    )
    save(model, output / "policy", env, metadata, "anchored_ppo")
    (output / "anchor-updates.json").write_text(json.dumps(model.anchor_log, indent=2) + "\n")
    result = evaluate_team(env, ModelTeam(model), 60, 610000)
    (output / "development.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"Final team development: {result['successes']}/60", flush=True)
    experiments = {
        name: {
            "sha256": hashlib.sha256((output / (name + ".zip")).read_bytes()).hexdigest(),
            "manifest": json.loads((output / (name + ".json")).read_text()),
        }
        for name in ("bc", "dagger", "policy")
        if (output / (name + ".zip")).exists()
    }
    (output / "experiments.json").write_text(json.dumps(experiments, indent=2) + "\n")
