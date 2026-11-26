"""Balanced imitation, DAgger and constrained PPO for the full scoring sequence."""

from dataclasses import asdict
import json
import os
from pathlib import Path
import time
import numpy as np
import torch
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback
from .dribble_refinement import AnchoredPPO
from .shoot import ShootTeacher, evaluate_shoot
from .shoot_contract import (
    SHOOT_CONTRACT,
    SHOT_SPEED,
    shoot_action,
    shoot_command,
    shoot_observation,
)


class ModelPolicy:
    def __init__(self, model):
        self.model = model

    def command(self, world, key, intent, cfg):
        action, _ = self.model.predict(
            shoot_observation(world, key, intent, cfg), deterministic=True
        )
        return shoot_command(action, cfg)


def groups(inputs, labels):
    firing = labels[:, 3] > 0
    held = inputs[:, 11] > 0.5
    return [np.flatnonzero(~held & ~firing), np.flatnonzero(held & ~firing), np.flatnonzero(firing)]


def collect(env, episodes, seed, model=None):
    teacher, rng = ShootTeacher(), np.random.default_rng(seed)
    x, y, records = [], [], []
    for i in range(episodes):
        obs, _ = env.reset(seed=seed + i)
        while True:
            label = shoot_action(teacher.command(env.world, env.key, env.intent, env.cfg), env.cfg)
            x.append(obs.copy())
            y.append(label)
            action = label
            if model is not None and rng.random() > 0.2:
                action, _ = model.predict(obs, deterministic=True)
            obs, _, done, truncated, info = env.step(action)
            if done or truncated:
                records.append({**info, "seed": seed + i})
                break
        if (i + 1) % 25 == 0:
            print(
                f"Scoring collection {i + 1}/{episodes}: {sum(r['is_success'] for r in records)} goals; {len(x)} samples",
                flush=True,
            )
    return np.asarray(x, np.float32), np.asarray(y, np.float32), records


def fit_actor(model, x, y, epochs, seed):
    rng = np.random.default_rng(seed)
    indices = groups(x, y)
    inputs, labels = torch.as_tensor(x), torch.as_tensor(y)
    optimizer = torch.optim.Adam(model.actor_parameters(), lr=6e-4)
    for epoch in range(epochs):
        samples = np.concatenate([rng.choice(g, 16384, replace=True) for g in indices if len(g)])
        rng.shuffle(samples)
        losses = []
        for batch in np.array_split(samples, max(1, len(samples) // 512)):
            mean = model.policy.get_distribution(inputs[batch]).distribution.mean
            loss = ((mean - labels[batch]) ** 2 * torch.tensor([2.0, 2.0, 1.0, 0.4])).mean()
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            losses.append(loss.item())
        if (epoch + 1) % 10 == 0:
            print(f"Scoring cloning {epoch + 1}/{epochs}: MSE {np.mean(losses):.7f}", flush=True)


def save(model, path, env, metadata, stage):
    path = Path(path)
    model.save(path)
    path.with_suffix(".json").write_text(
        json.dumps(
            {
                "contract": SHOOT_CONTRACT,
                "skill": "shoot",
                "config": asdict(env.cfg),
                "task": env.task_version,
                "backend": env.backend.name,
                "method": "Webots demonstration cloning + DAgger selection + anchored PPO",
                "stage": stage,
                "actual_steps": model.num_timesteps,
                "validated_for_hardware": False,
                "episode_steps": env.episode_steps,
                "minimum_carry": env.minimum_carry,
                "maximum_shot_distance": env.maximum_shot_distance,
                "shot_speed": SHOT_SPEED,
                "possession_sample_seconds": 0.005,
                **metadata,
            },
            indent=2,
        )
        + "\n"
    )


def train_shoot(env, output, steps=65536, seed=27):
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
        policy_kwargs={"net_arch": dict(pi=[128, 128], vf=[128, 128]), "log_std_init": -4.0},
    )
    metadata = {
        "seed": seed,
        "demonstration_episodes": demos,
        "dagger_episodes": dagger,
        "demonstration_seed_range": [200000, 200000 + demos - 1],
        "dagger_seed_range": [220000, 220000 + dagger - 1],
        "development_seed_range": [210000, 210059],
        "qualification_seed_range": [250000, 250199],
        "learning_rate": 3e-7,
        "anchor_max_normalized_action_delta": 0.01,
        "anchor_mean_square_delta": 1e-6,
    }
    x, y, records = collect(env, demos, 200000)
    (output / "teacher-collection.json").write_text(json.dumps(records, indent=2) + "\n")
    fit_actor(model, x, y, 80, seed)
    save(model, output / "bc", env, metadata, "cloned")
    bc = evaluate_shoot(env, ModelPolicy(model), 60, 210000)
    (output / "bc-development.json").write_text(json.dumps(bc, indent=2) + "\n")
    print(
        f"Cloned scoring development: {bc['successes']}/60; losses {bc['possession_loss_episodes']}; invalid shots {bc['invalid_shot_episodes']}",
        flush=True,
    )
    best = "bc"
    if dagger:
        dx, dy, records = collect(env, dagger, 220000, model)
        (output / "dagger-collection.json").write_text(json.dumps(records, indent=2) + "\n")
        x, y = np.concatenate([x, dx]), np.concatenate([y, dy])
        fit_actor(model, x, y, 50, seed + 1)
        save(model, output / "dagger", env, metadata, "dagger")
        candidate = evaluate_shoot(env, ModelPolicy(model), 60, 210000)
        (output / "dagger-development.json").write_text(json.dumps(candidate, indent=2) + "\n")
        print(
            f"DAgger scoring development: {candidate['successes']}/60; losses {candidate['possession_loss_episodes']}; invalid shots {candidate['invalid_shot_episodes']}",
            flush=True,
        )

        def rank(result):
            return (
                result["successes"],
                -result["possession_loss_episodes"],
                -(result["mean_success_seconds"] or 1000),
            )

        if rank(candidate) > rank(bc):
            best = "dagger"
    np.savez_compressed(output / "demonstrations.npz", observations=x, actions=y)
    metadata["anchor_reference"] = best
    print(f"Selected {best} on development; qualification still untouched", flush=True)
    model = AnchoredPPO.load(
        output / (best + ".zip"), env=Monitor(env), device="cpu", learning_rate=3e-7
    )
    model.prepare_anchor(x, seed, groups(x, y))
    start = time.monotonic()

    class Progress(BaseCallback):
        def _on_step(self):
            if self.num_timesteps % 8192 == 0:
                print(
                    f"Scoring anchored PPO {self.num_timesteps}/{steps}; {time.monotonic() - start:.1f}s",
                    flush=True,
                )
            return True

    model.learn(total_timesteps=steps, callback=Progress())
    metadata.update(
        anchor_updates=model.anchor_updates, anchor_rejected_updates=model.anchor_rejections
    )
    save(model, output / "policy", env, metadata, "anchored_ppo")
    (output / "anchor-updates.json").write_text(json.dumps(model.anchor_log, indent=2) + "\n")
    result = evaluate_shoot(env, ModelPolicy(model), 60, 210000)
    (output / "development.json").write_text(json.dumps(result, indent=2) + "\n")
    print(
        f"Final scoring development: {result['successes']}/60; losses {result['possession_loss_episodes']}; invalid shots {result['invalid_shot_episodes']}",
        flush=True,
    )
