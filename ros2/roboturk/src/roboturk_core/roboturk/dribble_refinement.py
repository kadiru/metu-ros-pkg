"""Conservative PPO refinement anchored to the successful cloned actor.

Alternates PPO updates with reference-policy regression on training observations.
Rejects an actor update if its fixed training probe exceeds the declared trust
bounds. This is a training constraint; inference is the ordinary single MLP.
"""

import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from .dribble import evaluate_dribble
from .dribble_training import ModelPolicy, save


class AnchoredPPO(PPO):
    def prepare_anchor(self, observations, seed, groups=None):
        self.anchor_rng = np.random.default_rng(seed)
        self.anchor_inputs = torch.as_tensor(observations, dtype=torch.float32)
        with torch.no_grad():
            self.anchor_targets = torch.cat(
                [
                    self.policy.get_distribution(x).distribution.mean
                    for x in self.anchor_inputs.split(4096)
                ]
            )
        held = observations[:, 11] >= 0.5
        near = np.linalg.norm(observations[:, 12:14], axis=1) * 5 < 0.4
        self.anchor_groups = (
            groups
            if groups is not None
            else [
                np.flatnonzero(~held),
                np.flatnonzero(held & ~near),
                np.flatnonzero(held & near),
            ]
        )
        if not self.anchor_groups or any(len(group) == 0 for group in self.anchor_groups):
            raise ValueError("Each anchor phase must contain training observations")
        self.anchor_probe = np.concatenate(
            [self.anchor_rng.choice(g, 4096, replace=True) for g in self.anchor_groups]
        )
        self.anchor_optimizer = torch.optim.Adam(self.actor_parameters(), lr=3e-5)
        self.anchor_updates = self.anchor_rejections = 0
        self.anchor_log = []

    def actor_parameters(self):
        return list(self.policy.mlp_extractor.policy_net.parameters()) + list(
            self.policy.action_net.parameters()
        )

    def _excluded_save_params(self):
        return super()._excluded_save_params() + [
            "anchor_rng",
            "anchor_inputs",
            "anchor_targets",
            "anchor_groups",
            "anchor_probe",
            "anchor_optimizer",
            "anchor_log",
        ]

    def train(self):
        previous = [p.detach().clone() for p in self.actor_parameters()]
        super().train()
        # Project back toward the original actor, balancing acquisition, carrying and braking.
        for _ in range(8):
            indices = np.concatenate(
                [self.anchor_rng.choice(g, 512, replace=True) for g in self.anchor_groups]
            )
            prediction = self.policy.get_distribution(self.anchor_inputs[indices]).distribution.mean
            loss = ((prediction - self.anchor_targets[indices]) ** 2).mean()
            self.anchor_optimizer.zero_grad()
            loss.backward()
            self.anchor_optimizer.step()
        with torch.no_grad():
            prediction = self.policy.get_distribution(
                self.anchor_inputs[self.anchor_probe]
            ).distribution.mean
            difference = prediction - self.anchor_targets[self.anchor_probe]
            mse, maximum = float((difference**2).mean()), float(difference.abs().max())
        rejected = (
            not math.isfinite(mse) or not math.isfinite(maximum) or mse > 1e-6 or maximum > 0.01
        )
        if rejected:
            with torch.no_grad():
                for parameter, old in zip(self.actor_parameters(), previous, strict=True):
                    parameter.copy_(old)
            self.anchor_optimizer.state.clear()
        self.anchor_updates += 1
        self.anchor_rejections += int(rejected)
        self.anchor_log.append(
            {
                "steps": self.num_timesteps,
                "probe_mse_before_rejection": mse,
                "probe_max_before_rejection": maximum,
                "actor_update_rejected": rejected,
            }
        )
        if self.anchor_updates % 16 == 0:
            print(
                f"Actor anchor: {self.anchor_updates} updates, {self.anchor_rejections} rejected; probe MSE {mse:.8f}, max {maximum:.5f}",
                flush=True,
            )


def refine_dribble(env, checkpoint, output, steps=65536, seed=18):
    torch.set_num_threads(1)
    checkpoint, output = Path(checkpoint), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    model = AnchoredPPO.load(checkpoint, env=Monitor(env), device="cpu", learning_rate=3e-7)
    model.set_random_seed(seed)
    data = np.load(checkpoint.parent / "demonstrations.npz")
    model.prepare_anchor(data["observations"], seed)
    metadata = json.loads(checkpoint.with_suffix(".json").read_text())
    for key in ("contract", "skill", "config", "task", "backend", "stage", "actual_steps"):
        metadata.pop(key, None)
    metadata.update(
        method="Webots cloning + PPO with reference-policy anchoring",
        seed=seed,
        parent_checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
        anchor_max_normalized_action_delta=0.01,
        anchor_mean_square_delta=1e-6,
        learning_rate=3e-7,
        anchor_projection_steps=8,
        anchor_reference="cloned actor; demonstration and DAgger observations only",
    )
    start = time.monotonic()

    class Progress(BaseCallback):
        def _on_step(self):
            if self.num_timesteps % 8192 == 0:
                print(
                    f"Anchored PPO {self.num_timesteps}/{steps}; {time.monotonic() - start:.1f}s",
                    flush=True,
                )
            return True

    model.learn(total_timesteps=steps, callback=Progress())
    metadata.update(
        anchor_updates=model.anchor_updates, anchor_rejected_updates=model.anchor_rejections
    )
    save(model, output / "policy", env, metadata, "anchored_ppo")
    (output / "anchor-updates.json").write_text(json.dumps(model.anchor_log, indent=2) + "\n")
    result = evaluate_dribble(env, ModelPolicy(model), 60, 110000)
    (output / "development.json").write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"Anchored PPO development: {result['successes']}/60; losses {result['possession_loss_episodes']}",
        flush=True,
    )
