"""CPU-only demonstration/DAgger/anchored-PPO protocol for explicit micro tasks."""

from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import time
import numpy as np
import torch
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback
from .dribble_refinement import AnchoredPPO


class Actor:
    def __init__(self, model):
        self.model = model

    def action(self, env):
        return self.model.predict(env.observation(), deterministic=True)[0]


def evaluate(env, actor, episodes, seed):
    records = []
    for i in range(episodes):
        env.reset(seed=seed + i)
        while True:
            _, _, done, truncated, info = env.step(actor.action(env))
            if done or truncated:
                records.append({**info, "seed": seed + i})
                break
        if (i + 1) % 20 == 0:
            print(
                f"{env.task_version}: {i + 1}/{episodes}, {sum(r['is_success'] for r in records)} successes",
                flush=True,
            )
    n, k, z = episodes, sum(r["is_success"] for r in records), 1.959963984540054
    p = k / n
    center = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return {
        "episodes": records,
        "successes": k,
        "success_rate": p,
        "wilson_95": [center - half, min(1, center + half)],
        "possession_loss_episodes": sum(r.get("lost_possession_checks", 0) > 0 for r in records),
        "mean_success_seconds": float(
            np.mean([r["elapsed_seconds"] for r in records if r["is_success"]])
        )
        if k
        else None,
    }


def phase_groups(x, y, kind):
    if kind == "keeper":
        fire = y[:, 3] > 0
        finish = x[:, 27] > 0.5
        indices = [
            np.flatnonzero(~finish & ~fire),
            np.flatnonzero(finish & ~fire),
            np.flatnonzero(fire),
        ]
    else:
        indices = [np.arange(len(x))]
    return [g for g in indices if len(g)]


def collect(env, teacher, episodes, seed, model=None):
    rng = np.random.default_rng(seed)
    x, y, records = [], [], []
    for i in range(episodes):
        obs, _ = env.reset(seed=seed + i)
        while True:
            label = teacher.action(env)
            x.append(obs.copy())
            y.append(label)
            action = (
                label
                if model is None or rng.random() < 0.2
                else model.predict(obs, deterministic=True)[0]
            )
            obs, _, done, truncated, info = env.step(action)
            if done or truncated:
                records.append({**info, "seed": seed + i})
                break
        if (i + 1) % 25 == 0:
            print(
                f"Collection {i + 1}/{episodes}: {sum(r['is_success'] for r in records)} successes; {len(x)} samples",
                flush=True,
            )
    return np.array(x, np.float32), np.array(y, np.float32), records


def fit(model, x, y, epochs, seed, kind):
    rng = np.random.default_rng(seed)
    groups = phase_groups(x, y, kind)
    inputs, labels = torch.as_tensor(x), torch.as_tensor(y)
    optimizer = torch.optim.Adam(model.actor_parameters(), lr=5e-4)
    weight = torch.tensor([2.0, 2.0, 1.0, 0.3]) if kind == "keeper" else 1.0
    for epoch in range(epochs):
        samples = np.concatenate([rng.choice(g, 16384, replace=True) for g in groups])
        rng.shuffle(samples)
        losses = []
        for batch in np.array_split(samples, max(1, len(samples) // 512)):
            mean = model.policy.get_distribution(inputs[batch]).distribution.mean
            loss = ((mean - labels[batch]) ** 2 * weight).mean()
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            losses.append(loss.item())
        if (epoch + 1) % 10 == 0:
            print(f"Clone {epoch + 1}/{epochs}: MSE {np.mean(losses):.8f}", flush=True)


def train(env, teacher, output, steps, seed, kind):
    torch.set_num_threads(1)
    torch.manual_seed(seed)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    start_seed = {"keeper": 900000, "block": 1000000, "cover": 1100000}[kind]
    demos = int(os.environ.get("ROBOTURK_DEMOS", "300" if kind == "keeper" else "200"))
    dagger = int(os.environ.get("ROBOTURK_DAGGER", "150" if kind == "keeper" else "100"))
    metadata = {
        "contract": env.contract,
        "config": asdict(env.cfg),
        "task": env.task_version,
        "backend": env.backend.name,
        "method": "Webots demonstration cloning + DAgger + anchored PPO",
        "seed": seed,
        "demonstration_seed_range": [start_seed, start_seed + demos - 1],
        "dagger_seed_range": [start_seed + 20000, start_seed + 20000 + dagger - 1],
        "development_seed_range": [start_seed + 10000, start_seed + 10059],
        "qualification_seed_range": [start_seed + 50000, start_seed + 50199],
        "validated_for_hardware": False,
        "learning_rate": 3e-7,
        "anchor_max_normalized_action_delta": 0.01,
        "anchor_mean_square_delta": 1e-6,
    }
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
        ent_coef=0,
        gamma=0.995,
        verbose=0,
        policy_kwargs={"net_arch": {"pi": [128, 128], "vf": [128, 128]}, "log_std_init": -4},
    )

    def save(stage):
        model.save(output / stage)
        (output / (stage + ".json")).write_text(
            json.dumps({**metadata, "stage": stage, "actual_steps": model.num_timesteps}, indent=2)
            + "\n"
        )

    def report(stage):
        result = evaluate(env, Acto…1103 tokens truncated…2,
                f"{phase} | {env.steps * env.cfg.dt + info['elapsed_seconds']:.2f} s",
                0.16,
                0.042,
                0x68FFB0,
            )
        self.label(
            6,
            f"Keeper: {env.backend.keeper.style.replace('_', ' ')} | saves {env.backend.contacts}",
            0.275,
            0.03,
        )


def record(supervisor, env, policy, shooter, output, episodes=6, seed=960000):
    output = Path(output)
    recorder = Recorder(supervisor, output, policy, shooter)
    supervisor.movieStartRecording(str(output.with_suffix(".mp4")), 1280, 720, 0, 85, 1, False)
    result = evaluate_team(env, policy, episodes, seed, shooter, recorder)
    supervisor.movieStopRecording()
    deadline = time.monotonic() + 60
    while not supervisor.movieIsReady() and time.monotonic() < deadline:
        if supervisor.step(20) == -1:
            raise RuntimeError("Webots closed during movie encoding")
    if not supervisor.movieIsReady() or supervisor.movieFailed():
        raise RuntimeError("Webots movie encoding failed or timed out")
    result.update(
        source="actual Webots movie recorder",
        policy_fallback=False,
        scripted_target_selection=True,
        scripted_role_handoff=True,
        outcome_display_seconds=0.6,
        possession_sample_seconds=0.005,
    )
    return result
