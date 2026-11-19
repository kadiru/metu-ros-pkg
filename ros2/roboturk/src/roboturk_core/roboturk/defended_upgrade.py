"""Defender transfer v2: attack-frame receiver control and fresh recovery seeds."""

import hashlib
import json
from pathlib import Path
import time
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from .dribble_refinement import AnchoredPPO
from .team import evaluate_team
from .defended import DEFENDED_CONTRACT, DefendedTeacher, augment_field_velocities, upgrade_actions
from .team_training import ModelTeam, collect, fit_actor, groups, save


def upgrade_defended(env, checkpoint, output, steps=65536, seed=77):
    torch.set_num_threads(1)
    torch.manual_seed(seed)
    checkpoint, output = Path(checkpoint), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    data = np.load(checkpoint.parent / "demonstrations.npz")
    x, y = data["observations"], data["actions"]
    if x.shape[1] != 52:
        raise ValueError("Expected original 52-dimensional training observations")
    y = upgrade_actions(x, y)
    x = augment_field_velocities(x)
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
        policy_kwargs={"net_arch": dict(pi=[192, 192], vf=[192, 192]), "log_std_init": -4.0},
        verbose=0,
    )
    previous = PPO.load(checkpoint, device="cpu").policy.state_dict()
    current = model.policy.state_dict()
    for name, value in current.items():
        old = previous[name]
        if old.shape == value.shape:
            current[name] = old
        elif name.endswith("policy_net.0.weight") or name.endswith("value_net.0.weight"):
            value.zero_()
            value[:, :52] = old
        else:
            raise ValueError(f"Unsupported warm-start shape: {name}")
    current["action_net.weight"][4:6].zero_()
    current["action_net.bias"][4:6].zero_()
    model.policy.load_state_dict(current)
    metadata = {
        "contract": DEFENDED_CONTRACT,
        "opponent": "reactive_lane_defender_v1",
        "seed": seed,
        "source_checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
        "reused_demonstration_seed_range": [600000, 600299],
        "reused_dagger_seed_range": [620000, 620149],
        "new_dagger_seed_range": [720000, 720149],
        "development_seed_range": [710000, 710059],
        "qualification_seed_range": [750000, 750199],
        "demo_seed_range": [760000, 760005],
        "learning_rate": 3e-7,
        "anchor_max_normalized_action_delta": 0.01,
        "anchor_mean_square_delta": 1e-6,
        "method": "Attack-frame receiver warm start + new DAgger + anchored PPO",
    }
    fit_actor(model, x, y, 100, seed)
    save(model, output / "bc", env, metadata, "frame_refit")
    bc = evaluate_team(env, ModelTeam(model), 60, 710000)
    (output / "bc-development.json").write_text(json.dumps(bc, indent=2) + "\n")
    print(
        f"Frame-refit development: {bc['successes']}/60; losses {bc['possession_loss_episodes']}",
        flush=True,
    )
    dx, dy, records = collect(env, 150, 720000, model, teacher=DefendedTeacher())
    (output / "dagger-collection.json").write_text(json.dumps(records, indent=2) + "\n")
    x, y = np.concatenate([x, dx]), np.concatenate([y, dy])
    fit_actor(model, x, y, 60, seed + 1)
    save(model, output / "dagger", env, metadata, "dagger")
    candidate = evaluate_team(env, ModelTeam(model), 60, 710000)
    (output / "dagger-development.json").write_text(json.dumps(candidate, indent=2) + "\n")
    print(
        f"V2 DAgger development: {candidate['successes']}/60; losses {candidate['possession_loss_episodes']}",
        flush=True,
    )

    def rank(r):
        return (
            r["successes"],
            -r["possession_loss_episodes"],
            -(r["mean_success_seconds"] or 1000),
        )

    best = "dagger" if rank(candidate) > rank(bc) else "bc"
    metadata["anchor_reference"] = best
    np.savez_compressed(output / "demonstrations.npz", observations=x, actions=y)
    print(f"Selected {best}; new qualification set untouched", flush=True)
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
                    f"V2 anchored PPO {self.num_timesteps}/{steps}; {time.monotonic() - start:.1f}s",
                    flush=True,
                )
            return True

    model.learn(total_timesteps=steps, callback=Progress())
    metadata.update(
        anchor_updates=model.anchor_updates, anchor_rejected_updates=model.anchor_rejections
    )
    save(model, output / "policy", env, metadata, "anchored_ppo")
    (output / "anchor-updates.json").write_text(json.dumps(model.anchor_log, indent=2) + "\n")
    result = evaluate_team(env, ModelTeam(model), 60, 710000)
    (output / "development.json").write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"Final V2 development: {result['successes']}/60; losses {result['possession_loss_episodes']}",
        flush=True,
    )

    experiments = {
        name: {
            "sha256": hashlib.sha256((output / (name + ".zip")).read_bytes()).hexdigest(),
            "manifest": json.loads((output / (name + ".json")).read_text()),
        }
        for name in ("bc", "dagger", "policy")
    }
    (output / "experiments.json").write_text(json.dumps(experiments, indent=2) + "\n")
