"""Balanced demonstration/DAgger training and constrained PPO for two robots."""

from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import time
import numpy as np
import torch
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback
from .dribble_refinement import AnchoredPPO
from .team import TransferTeacher, evaluate_team
from .team_contract import TEAM_CONTRACT, PASS_SPEED


class ModelTeam:
    def __init__(self, model):
        self.model = model

    def action(self, env):
        return self.model.predict(env.observation(), deterministic=True)[0]


def groups(x, y):
    passed = x[:, 31] > 0.5
    firing = y[:, 3] > 0
    held = x[:, 28] > 0.5
    return [
        np.flatnonzero(~passed & ~firing),
        np.flatnonzero(firing),
        np.flatnonzero(passed & ~held),
        np.flatnonzero(passed & held),
    ]


def collect(env, episodes, seed, model=None, teacher=None):
    teacher, rng = teacher or TransferTeacher(), np.random.default_rng(seed)
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
                f"Team collection {i + 1}/{episodes}: {sum(r['is_success'] for r in records)} received; {len(x)} samples",
                flush=True,
            )
    return np.asarray(x, np.float32), np.asarray(y, np.float32), records


def fit_actor(model, x, y, epochs, seed):
    rng = np.random.default_rng(seed)
    indices = groups(x, y)
    inputs, labels = torch.as_tensor(x), torch.as_tensor(y)
    optimizer = torch.optim.Adam(model.actor_parameters(), lr=6e-4)
    for epoch in range(epochs):
        batch_indices = np.concatenate(
            [rng.choice(g, 12288, replace=True) for g in indices if len(g)]
        )
        rng.shuffle(batch_indices)
        losses = []
        for batch in np.array_split(batch_indices, max(1, len(batch_indices) // 512)):
            pred = model.policy.get_distribution(inputs[batch]).distribution.mean
            loss = (
                (pred - labels[batch]) ** 2 * torch.tensor([2.0, 2.0, 1.0, 0.4, 2.0, 2.0, 1.0])
            ).mean()
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            losses.append(loss.item())
        if (epoch + 1) % 10 == 0:
            print(f"Team cloning {epoch + 1}/{epochs}: MSE {np.mean(losses):.7f}", flush=True)


def save(model, path, env, metadata, stage):
    path = Path(path)
    model.save(path)
    path.with_suffix(".json").write_text(
        json.dumps(
            {
                "contract": TEAM_CONTRACT,
                "skill": "pass_receive_team",
                "config": asdict(env.cfg),
                "task": env.task_version,
                "backend": env.backend.name,
                "stage": stage,
                "method": "Webots cloning + DAgger selection + anchored PPO",
                "actual_steps": model.num_timesteps,
                "pass_speed": PASS_SPEED,
                "centralized": True,
                "validated_for_hardware": False,
                **metadata,
            },
            indent=2,
        )
        + "\n"
    )


def train_team(env, output, steps=65536, seed=47):
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
    metadata = {
        "seed": seed,
        "demonstration_seed_range": [400000, 400000 + demos - 1],
        "dagger_seed_range": [420000, 420000 + dagger - 1],
        "development_seed_range": [410000, 410059],
        "qualification_seed_range": [450000, 450199],
        "demo_seed_range": [460000, 460005],
        "learning_rate": 3e-7,
        "anchor_max_normalized_action_delta": 0.01,
        "anchor_mean_square_delta": 1e-6,
    }
    x, y, records = collect(env, demos, 400000)
    (output / "teacher-collection.json").write_text(json.dumps(records, indent=2) + "\n")
    fit_actor(model, x, y, 80, seed)
    save(model, output / "bc", env, metadata, "cloned")
    bc = evaluate_team(env, ModelTeam(model), 60, 410000)
    (output / "bc-development.json").write_text(json.dumps(bc, indent=2) + "\n")
    print(f"Cloned team development: {bc['successes']}/60", flush=True)
    best = "bc"
    best_result = bc
    if dagger:
        dx, dy, records = collect(env, dagger, 420000, model)
        (output / "dagger-collection.json").write_text(json.dumps(records, indent=2) + "\n")
        x, y = np.concatenate([x, dx]), np.concatenate([y, dy])
        fit_actor(model, x, y, 50, seed + 1)
        save(model, output / "dagger", env, metadata, "dagger")
        result = evaluate_team(env, ModelTeam(model), 60, 410000)
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
    result = evaluate_team(env, ModelTeam(model), 60, 410000)
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
