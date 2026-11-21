"""Demonstration warm start, DAgger recovery examples, then PPO in the same Webots backend."""

from dataclasses import asdict
import json
from pathlib import Path
import time
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from .capture import CaptureTeacher, evaluate_capture
from .capture_contract import CAPTURE_CONTRACT, capture_action, capture_command, capture_observation


class ModelPolicy:
    def __init__(self, model):
        self.model = model

    def command(self, world, key, intent, cfg):
        action, _ = self.model.predict(capture_observation(world, key, cfg), deterministic=True)
        return capture_command(action, cfg)


def collect(env, episodes, seed, model=None):
    teacher = CaptureTeacher()
    inputs, labels = [], []
    rng = np.random.default_rng(seed)
    successes = 0
    for i in range(episodes):
        obs, _ = env.reset(seed=seed + i)
        while True:
            expert = capture_action(
                teacher.command(env.world, env.key, env.intent, env.cfg), env.cfg
            )
            inputs.append(obs.copy())
            labels.append(expert)
            action = expert
            if model is not None and rng.random() > 0.2:
                action, _ = model.predict(obs, deterministic=True)
            obs, _, done, truncated, info = env.step(action)
            if done or truncated:
                successes += int(info["is_success"])
                break
        if (i + 1) % 25 == 0:
            print(
                f"Demonstrations {i + 1}/{episodes}, successes {successes}, samples {len(inputs)}",
                flush=True,
            )
    return np.asarray(inputs, np.float32), np.asarray(labels, np.float32)


def clone(model, inputs, labels, epochs, seed):
    rng = np.random.default_rng(seed)
    actor_parameters = list(model.policy.mlp_extractor.policy_net.parameters()) + list(
        model.policy.action_net.parameters()
    )
    optimizer = torch.optim.Adam(actor_parameters, lr=1e-3)
    x, y = torch.as_tensor(inputs), torch.as_tensor(labels)
    for epoch in range(epochs):
        losses = []
        for indices in np.array_split(rng.permutation(len(inputs)), max(1, len(inputs) // 512)):
            prediction = model.policy.get_distribution(x[indices]).distribution.mean
            # Balance angular and translational accuracy; dribbler remains a learned output.
            loss = ((prediction - y[indices]) ** 2 * torch.tensor([2.0, 2.0, 1.0, 0.5])).mean()
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            losses.append(loss.item())
        if (epoch + 1) % 10 == 0:
            print(f"Cloning epoch {epoch + 1}: MSE {np.mean(losses):.6f}", flush=True)


def save(model, path, env, metadata):
    path = Path(path)
    model.save(path)
    path.with_suffix(".json").write_text(
        json.dumps(
            {
                "contract": CAPTURE_CONTRACT,
                "skill": "chase",
                "config": asdict(env.cfg),
                "task": env.task_version,
                "backend": env.backend.name,
                "method": "Webots demonstration cloning + DAgger + PPO",
                "actual_steps": model.num_timesteps,
                "validated_for_hardware": False,
                **metadata,
            },
            indent=2,
        )
        + "\n"
    )


def train_capture(env, output, steps=131072, seed=7, demonstrations=300, dagger=150):
    torch.set_num_threads(1)
    torch.manual_seed(seed)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    model = PPO(
        "MlpPolicy",
        Monitor(env),
        seed=seed,
        device="cpu",
        n_steps=1024,
        batch_size=256,
        n_epochs=5,
        learning_rate=1e-5,
        clip_range=0.1,
        ent_coef=0.0,
        gamma=0.99,
        verbose=0,
        policy_kwargs={"net_arch": dict(pi=[128, 128], vf=[128, 128]), "log_std_init": -3.0},
    )
    metadata = {
        "seed": seed,
        "demonstration_episodes": demonstrations,
        "dagger_episodes": dagger,
        "training_seed_ranges": [[0, demonstrations - 1], [20000, 20000 + dagger - 1]],
        "development_seed_range": [10000, 10049],
        "qualification_seed_range": [50000, 50199],
    }
    inputs, labels = collect(env, demonstrations, 0)
    clone(model, inputs, labels, 60, seed)
    save(model, output / "bc", env, metadata)
    warm = evaluate_capture(env, ModelPolicy(model), 50, 10000)
    (output / "bc-evaluation.json").write_text(json.dumps(warm, indent=2) + "\n")
    print(f"Warm-start development success: {warm['success_rate']}", flush=True)
    if dagger:
        more_x, more_y = collect(env, dagger, 20000, model)
        inputs, labels = np.concatenate([inputs, more_x]), np.concatenate([labels, more_y])
        clone(model, inputs, labels, 40, seed + 1)
    save(model, output / "dagger", env, metadata)
    np.savez_compressed(output / "demonstrations.npz", observations=inputs, actions=labels)
    start = time.monotonic()

    class Progress(BaseCallback):
        def _on_step(self):
            if self.num_timesteps % 8192 == 0:
                print(
                    f"PPO {self.num_timesteps}/{steps}; {time.monotonic() - start:.1f}s", flush=True
                )
            if self.num_timesteps % 32768 == 0:
                save(self.model, output / f"ppo_{self.num_timesteps}", env, metadata)
            return True

    model.learn(total_timesteps=steps, callback=Progress())
    save(model, output / "policy", env, metadata)
    result = evaluate_capture(env, ModelPolicy(model), 50, 10000)
    (output / "development.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"PPO development success: {result['success_rate']}", flush=True)
