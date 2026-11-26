"""Native demonstration/DAgger preparation and bounded PPO for ball options."""

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import torch
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback
from .model import compatible_policy_config
from .dribble_refinement import AnchoredPPO
from .tdp_options import CONTRACT, OBSERVATIONS, KINDS, BallOptionTeacher


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, separators=(",", ":")) + "\n")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def actor_hash(model):
    h = hashlib.sha256()
    for p in model.actor_parameters():
        h.update(p.detach().cpu().numpy().tobytes())
    return h.hexdigest()


def descriptor(path, model):
    return {
        "path": str(path),
        "sha256": digest(path),
        "metadata_sha256": digest(Path(path).with_suffix(".json")),
        "actor_sha256": actor_hash(model),
        "contract": CONTRACT,
    }


def load(path, env):
    metadata = json.loads(Path(path).with_suffix(".json").read_text())
    assert metadata["contract"] == CONTRACT and metadata["kind"] == env.kind
    assert compatible_policy_config(metadata["config"], env.cfg)
    assert metadata["backend"] == "webots_ideal_body_velocity"
    model = AnchoredPPO.load(path, device="cpu")
    assert model.observation_space.shape == (OBSERVATIONS,) and model.action_space.shape == (4,)
    assert actor_hash(model) == metadata["actor_sha256"]
    return model


class OptionActor:
    def __init__(self, model):
        self.model = model

    def action(self, env):
        return self.model.predict(env.observation(), deterministic=True)[0]


def groups(x, y):
    fire = y[:, 3] > 0
    near = np.linalg.norm(x[:, :2], axis=1) < 0.08
    held = x[:, 11] > 0.5
    return [
        g
        for g in (
            np.flatnonzero(fire),
            np.flatnonzero(held & ~fire),
            np.flatnonzero(near & ~held),
            np.flatnonzero(~near & ~held),
        )
        if len(g)
    ]


def collect(env, count, seed, model=None):
    rng = np.random.default_rng(seed)
    x, y, records = [], [], []
    teacher = BallOptionTeacher()
    for i in range(count):
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
                # Full 5ms traces are reserved for evaluation; training data and
                # actual event/initial-state receipts remain retained separately.
                records.append({k: v for k, v in info.items() if k != "samples"})
                break
        if (i + 1) % 25 == 0:
            print(
                "collect",
                env.kind,
                i + 1,
                count,
                sum(r["is_success"] for r in records),
                len(x),
                flush=True,
            )
    return np.array(x, np.float32), np.array(y, np.float32), records


def fit(model, x, y, epochs, seed):
    rng = np.random.default_rng(seed)
    balanced = groups(x, y)
    inputs, labels = torch.as_tensor(x), torch.as_tensor(y)
    optimizer = torch.optim.Adam(model.actor_parameters(), lr=3e-4)
    weights = torch.tensor([8.0, 8.0, 3.0, 1.0])
    for epoch in range(epochs):
        selected = np.concatenate([rng.choice(g, 8192, replace=True) for g in balanced])
        rng.shuffle(selected)
        losses = []
        for indices in np.array_split(selected, max(1, len(selected) // 512)):
            prediction = model.policy.get_distribution(inputs[indices]).distribution.mean
            loss = ((prediction - labels[indices]) ** 2 * weights).mean()
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            losses.append(loss.item())
        if (epoch + 1) % 20 == 0:
            print("fit", epoch + 1, epochs, float(np.mean(losses)), flush=True)
    model.policy.optimizer.state.clear()


def train(env, output, steps, seed, evaluate):
    torch.set_num_threads(1)
    torch.manual_seed(seed)
    output = Path(output)
    if output.exists():
        raise ValueError("Training output already exists")
    output.mkdir(parents=True)
    base = 5000000 + 200000 * KINDS.index(env.kind)
    metadata = {
        "contract": CONTRACT,
        "kind": env.kind,
        "config": asdict(env.cfg),
        "backend": env.backend.name,
        "backend_contract": getattr(env.backend, "contract", None),
        "training_seed": seed,
        "method": "native demonstrations + mixed DAgger + anchored PPO",
        "teacher_seeds": [base, base + 199],
        "dagger_seeds": [base + 20000, base + 20099],
        "development_seeds": [base + 10000, base + 10059],
        "ppo_seeds_base": base + 30000,
        "qualification_seeds": [base + 50000, base + 50199],
        "demo_seeds": [base + 60000, base + 60003],
        "learning_rate": 3e-7,
        "anchor_mse": 1e-6,
        "anchor_max_delta": 0.01,
        "neural_motor_outputs": ["body_vx", "body_vy", "omega", "release"],
        "fixed_rules": [
            "receive disables kicker",
            "dribbler enabled except firing",
            "3.5m/s release",
            "stationary scripted pass server or clearance obstacle",
        ],
        "runtime_teacher": False,
        "runtime_action_projection": False,
        "validated_for_hardware": False,
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
        policy_kwargs={"net_arch": {"pi": [128, 128], "vf": [128, 128]}, "log_std_init": -4.5},
    )

    def save(stage):
        model.save(output / stage)
        write(
            output / (stage + ".json"),
            {
                **metadata,
                "stage": stage,
                "actual_steps": model.num_timesteps,
                "actor_sha256": actor_hash(model),
            },
        )

    def report(stage):
        result = evaluate(env, OptionActor(model), 60, base + 10000)
        result["checkpoint"] = descriptor(output / (stage + ".zip"), model)
        write(output / (stage + "-development.json"), result)
        return result["successes"]

    x, y, records = collect(env, 200, base)
    write(output / "teacher-collection.json", records)
    fit(model, x, y, 120, seed)
    save("bc")
    bc = report("bc")
    dx, dy, records = collect(env, 100, base + 20000, model)
    write(output / "dagger-collection.json", records)
    x, y = np.concatenate([x, dx]), np.concatenate([y, dy])
    np.savez_compressed(output / "demonstrations.npz", observations=x, actions=y)
    metadata["dataset_sha256"] = digest(output / "demonstrations.npz")
    fit(model, x, y, 100, seed + 1)
    save("dagger")
    dagger = report("dagger")
    chosen = "dagger" if dagger >= bc else "bc"
    model = AnchoredPPO.load(
        output / (chosen + ".zip"), env=Monitor(env), device="cpu", learning_rate=3e-7
    )
    model.set_random_seed(seed)
    before = actor_hash(model)
    metadata["anchor_reference"] = chosen
    model.prepare_anchor(x, seed, groups(x, y))
    for group in model.anchor_optimizer.param_groups:
        group["lr"] = 3e-6
    begin = time.monotonic()
    reset_start = len(env.native_resets)
    completed_rollouts = []

    class Progress(BaseCallback):
        def _on_step(self):
            if self.locals["dones"][0]:
                completed_rollouts.append(
                    {
                        k: v
                        for k, v in self.locals["infos"][0].items()
                        if k not in ("samples", "terminal_observation")
                    }
                )
            if self.num_timesteps % 8192 == 0:
                print(
                    "ppo",
                    env.kind,
                    self.num_timesteps,
                    steps,
                    round(time.monotonic() - begin, 1),
                    flush=True,
                )
            return True

    model.learn(total_timesteps=steps, callback=Progress())
    write(output / "ppo-resets.json", env.native_resets[reset_start:])
    write(output / "ppo-rollouts.json", completed_rollouts)
    metadata.update(
        actor_before_ppo=before,
        anchor_updates=model.anchor_updates,
        anchor_rejections=model.anchor_rejections,
    )
    save("policy")
    write(output / "anchor-updates.json", model.anchor_log)
    final = report("policy")
    write(
        output / "training-summary.json",
        {
            "kind": env.kind,
            "training_seed": seed,
            "bc_successes": bc,
            "dagger_successes": dagger,
            "final_successes": final,
            "development_count": 60,
            "ppo_steps": model.num_timesteps,
            "accepted_updates": model.anchor_updates - model.anchor_rejections,
            "actor_changed": before != actor_hash(model),
            "dataset_sha256": metadata["dataset_sha256"],
            "selection": "final PPO checkpoint; requires development pass before reserved qualification",
        },
    )
