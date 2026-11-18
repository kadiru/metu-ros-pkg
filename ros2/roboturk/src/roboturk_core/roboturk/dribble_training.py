"""Demonstration warm start and PPO for the goal-conditioned capture/carry policy."""

from dataclasses import asdict
import json
import os
from pathlib import Path
import time
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from .capture_contract import capture_action, capture_command
from .dribble import DribbleTeacher, evaluate_dribble
from .dribble_contract import DRIBBLE_CONTRACT, dribble_observation


class ModelPolicy:
    def __init__(self, model):
        self.model = model

    def command(self, world, key, intent, cfg):
        action, _ = self.model.predict(
            dribble_observation(world, key, intent, cfg), deterministic=True
        )
        return capture_command(action, cfg)


def collect(env, episodes, seed, model=None):
    teacher, rng = DribbleTeacher(), np.random.default_rng(seed)
    inputs, labels, records = [], [], []
    for i in range(episodes):
        obs, _ = env.reset(seed=seed + i)
        while True:
            label = capture_action(
                teacher.command(env.world, env.key, env.intent, env.cfg), env.cfg
            )
            inputs.append(obs.copy())
            labels.append(label)
            action = label
            if model is not None and rng.random() > 0.2:
                action, _ = model.predict(obs, deterministic=True)
            obs, _, done, truncated, info = env.step(action)
            if done or truncated:
                records.append({**info, "seed": seed + i})
                break
        if (i + 1) % 25 == 0:
            print(
                f"Dribble collection {i + 1}/{episodes}: {sum(r['is_success'] for r in records)} successes; {len(inputs)} samples",
                flush=True,
            )
    return np.asarray(inputs, np.float32), np.asarray(labels, np.float32), records


def fit_actor(model, inputs, labels, epochs, seed):
    rng = np.random.default_rng(seed)
    parameters = list(model.policy.mlp_extractor.policy_net.parameters()) + list(
        model.policy.action_net.parameters()
    )
    optimizer = torch.optim.Adam(parameters, lr=8e-4)
    x, y = torch.as_tensor(inputs), torch.as_tensor(labels)
    groups = [np.flatnonzero(inputs[:, 11] < 0.5), np.flatnonzero(inputs[:, 11] >= 0.5)]
    for epoch in range(epochs):
        # Long carry episodes otherwise overwhelm the short acquisition phase.
        indices = np.concatenate([rng.choice(g, 25600, replace=True) for g in groups if len(g)])
        rng.shuffle(indices)
        losses = []
        for batch in np.array_split(indices, max(1, len(indices) // 512)):
            mean = model.policy.get_distribution(x[batch]).distribution.mean
            loss = ((mean - y[batch]) ** 2 * torch.tensor([2.0, 2.0, 1.0, 0.5])).mean()
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            losses.append(loss.item())
        if (epoch + 1) % 10 == 0:
            print(f"Dribble cloning {epoch + 1}/{epochs}: MSE {np.mean(losses):.7f}", flush=True)


def save(model, path, env, metadata, stage):
    path = Path(path)
    model.save(path)
    path.with_suffix(".json").write_text(
        json.dumps(
            {
                "contract": DRIBBLE_CONTRACT,
                "skill": "dribble",
                "config": asdict(env.cfg),
                "task": env.task_version,
                "backend": env.backend.name,
                "method": "Webots demonstration cloning + DAgger + PPO",
                "stage": stage,
                "actual_steps": model.num_timesteps,
                "validated_for_hardware": False,
                "episode_steps": env.episode_steps,
                "target_radius": env.target_radius,
                "arrival_hold_steps": env.arrival_hold_steps,
                "possession_sample_seconds": 0.005,
                **metadata,
            },
            indent=2,
        )
        + "\n"
    )


def train_dribble(env, output, steps=131072, seed=17):
    torch.set_num_threads(1)
    torch.manual_seed(seed)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    demonstrations = int(os.environ.get("ROBOTURK_DEMOS", "300"))
    dagger = int(os.environ.get("ROBOTURK_DAGGER", "150"))
    model = PPO(
        "MlpPolicy",
        Monitor(env),
        seed=seed,
        device="cpu",
        n_steps=1024,
        batch_size=256,
        n_epochs=5,
        learning_rate=3e-6,
        clip_range=0.1,
        ent_coef=0.0,
        gamma=0.995,
        verbose=0,
        policy_kwargs={"net_arch": dict(pi=[128, 128], vf=[128, 128]), "log_std_init": -4.0},
    )
    metadata = {
        "seed": seed,
        "demonstration_episodes": demonstrations,
        "dagger_episodes": dagger,
        "demonstration_seed_range": [100000, 100000 + demonstrations - 1],
        "dagger_seed_range": [120000, 120000 + dagger - 1],
        "development_seed_range": [110000, 110059],
        "qualification_seed_range": [150000, 150199],
    }
    x, y, records = collect(env, demonstrations, 100000)
    (output / "teacher-collection.json").write_text(json.dumps(records, indent=2) + "\n")
    fit_actor(model, x, y, 80, seed)
    save(model, output / "bc", env, metadata, "cloned")
    result = evaluate_dribble(env, ModelPolicy(model), 60, 110000)
    (output / "bc-development.json").write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"Cloned development: {result['successes']}/60; losses {result['possession_loss_episodes']}",
        flush=True,
    )
    if dagger:
        dx, dy, records = collect(env, dagger, 120000, model)
        (output / "dagger-collection.json").write_text(json.dumps(records, indent=2) + "\n")
        x, y = np.concatenate([x, dx]), np.concatenate([y, dy])
        fit_actor(model, x, y, 50, seed + 1)
    np.savez_compressed(output / "demonstrations.npz", observations=x, actions=y)
    save(model, output / "dagger", env, metadata, "dagger")
    result = evaluate_dribble(env, ModelPolicy(model), 60, 110000)
    (output / "dagger-development.json").write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"DAgger development: {result['successes']}/60; losses {result['possession_loss_episodes']}",
        flush=True,
    )
    start = time.monotonic()

    class Progress(BaseCallback):
        def _on_step(self):
            if self.num_timesteps % 8192 == 0:
                print(
                    f"Dribble PPO {self.num_timesteps}/{steps}; {time.monotonic() - start:.1f}s",
                    flush=True,
                )
            if self.num_timesteps % 32768 == 0:
                save(self.model, output / f"ppo_{self.num_timesteps}", env, metadata, "ppo")
            return True

    model.learn(total_timesteps=steps, callback=Progress())
    save(model, output / "policy", env, metadata, "ppo")
    result = evaluate_dribble(env, ModelPolicy(model), 60, 110000)
    (output / "development.json").write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"PPO development: {result['successes']}/60; losses {result['possession_loss_episodes']}",
        flush=True,
    )
