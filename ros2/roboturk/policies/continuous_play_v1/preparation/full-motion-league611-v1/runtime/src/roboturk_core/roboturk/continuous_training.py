"""Native collection, explicit imitation initialization and historical self-play."""

from dataclasses import asdict
from pathlib import Path
import hashlib
import json
import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
from .continuous import ContinuousMatch, CONTRACT, OBSERVATIONS, ACTIONS, summary
from .continuous_teacher import ContinuousTeacher
from .self_play_training import SelfPlayPPO, actor_hash, write
from .model import Config


def digest(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def load(path):
    meta = json.loads(Path(path).with_suffix(".json").read_text())
    if meta["contract"] != CONTRACT:
        raise ValueError("Wrong continuous contract")
    if meta["config"] != asdict(Config()):
        raise ValueError("Wrong continuous physics configuration")
    if "checkpoint_sha256" in meta and digest(path) != meta["checkpoint_sha256"]:
        raise ValueError("Changed checkpoint bytes")
    model = PPO.load(path, device="cpu")
    if model.observation_space.shape != (OBSERVATIONS,) or model.action_space.shape != (ACTIONS,):
        raise ValueError("Wrong model shape")
    if actor_hash(model) != meta["actor_sha256"]:
        raise ValueError("Changed actor")
    return model


def descriptor(path, model):
    return {
        "path": str(path),
        "sha256": digest(path),
        "metadata_sha256": digest(Path(path).with_suffix(".json")),
        "actor_sha256": actor_hash(model),
        "contract": CONTRACT,
    }


def save(model, path, metadata):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    model.save(path)
    write(
        path.with_suffix(".json"),
        {
            **metadata,
            "contract": CONTRACT,
            "config": asdict(Config()),
            "actor_sha256": actor_hash(model),
            "checkpoint_sha256": digest(path),
            "hardware_validated": False,
        },
    )
    return {"path": str(path), "sha256": digest(path), "actor_sha256": actor_hash(model)}


def evaluate(backend, blue, yellow, count, seed, duration, recorder=None, trace=True):
    match = ContinuousMatch(backend, duration, trace=trace)
    records = []
    teacher = ContinuousTeacher() if blue is None else None
    for i in range(count):
        match.reset(seed + i)
        inference_batches = {"blue": 0, "yellow": 0}
        if recorder:
            recorder.begin(i, match)
        while not match.finished:
            match.restart_if_needed()
            obs = match.observations()
            if blue is None and yellow is None:
                actions = teacher.actions(match)
            else:
                if blue is None or yellow is None:
                    raise ValueError("No mixed analytic deployment")
                a = blue.predict(obs[:2], deterministic=True)[0]
                inference_batches["blue"] += 1
                b = yellow.predict(obs[2:], deterministic=True)[0]
                inference_batches["yellow"] += 1
                actions = np.concatenate([a, b])
            match.step(actions)
            if recorder:
                recorder.step(match)
        records.append({**match.result(), "neural_inference_batches": inference_batches})
        print("Continuous evaluation", i + 1, count, summary(records), flush=True)
    return {
        "backend": backend.name,
        "control": "training_teacher" if blue is None else "neural",
        "contract": CONTRACT,
        "duration": duration,
        "contact_terminates": False,
        "runtime_action_projection": False,
        "matches": records,
        "summary": summary(records),
    }


def collect(backend, output, count, seed, duration, model=None, mix=0):
    out = Path(output)
    out.mkdir(parents=True, exist_ok=True)
    match = ContinuousMatch(backend, duration)
    teacher = ContinuousTeacher()
    rng = np.random.default_rng(seed)
    xs = []
    ys = []
    records = []
    for i in range(count):
        match.reset(seed + i)
        while not match.finished:
            match.restart_if_needed()
            x = match.observations()
            y = teacher.actions(match)
            if match.steps % 2 == 0 or np.any(
                y[np.arange(4), 10 + 4 * np.argmax(y[:, :7], axis=1)] > 0
            ):
                xs.extend(x)
                ys.extend(y)
            action = y.copy()
            if model is not None and mix:
                neural = model.predict(x, deterministic=True)[0]
                mask = rng.random(4) < mix
                action[mask] = neural[mask]
            match.step(action)
        records.append(match.result())
        if (i + 1) % 5 == 0:
            print("Continuous collection", i + 1, count, summary(records), flush=True)
    x, y = np.asarray(xs, np.float32), np.asarray(ys, np.float32)
    np.savez_compressed(out / "data.npz", observations=x, actions=y)
    write(
        out / "collection.json",
        {
            "backend": backend.name,
            "control": (
                "training_teacher"
                if not mix
                else "neural_rollouts_teacher_labels"
                if mix == 1
                else f"mixed_neural_{mix}"
            ),
            "label_teacher_sha256": digest(Path(__file__).with_name("continuous_teacher.py")),
            "model_actor_sha256": actor_hash(model) if model is not None else None,
            "count": count,
            "seed": seed,
            "mix": mix,
            "matches": records,
            "summary": summary(records),
            "observations": len(x),
            "data_sha256": digest(out / "data.npz"),
        },
    )
    return x, y


class ShapeEnv(gym.Env):
    observation_space = gym.spaces.Box(-np.inf, np.inf, (OBSERVATIONS,), dtype=np.float32)
    action_space = gym.spaces.Box(-1, 1, (ACTIONS,), dtype=np.float32)


def fit(output, seed, epochs, source=None, learning_rate=3e-4):
    out = Path(output)
    data = np.load(out / "data.npz")
    x, y = data["observations"], data["actions"]
    model = SelfPlayPPO(
        "MlpPolicy",
        ShapeEnv(),
        seed=seed,
        device="cpu",
        n_steps=1024,
        batch_size=256,
        n_epochs=5,
        learning_rate=2e-6,
        gamma=0.995,
        clip_range=0.1,
        policy_kwargs={"net_arch": [256, 256], "log_std_init": -4.5},
    )
    if source:
        meta = json.loads(Path(source).with_suffix(".json").read_text())
        parent = PPO.load(source, device="cpu")
        if (
            meta["contract"]
            not in (CONTRACT, "roboturk.continuous_robot.v2", "roboturk.continuous_robot.v3")
            or meta["actor_sha256"] != actor_hash(parent)
            or meta["config"] != asdict(Config())
            or parent.action_space.shape != (ACTIONS,)
        ):
            raise ValueError("Invalid training parent")
        state = parent.policy.state_dict()
        target = model.policy.state_dict()
        for key, value in state.items():
            if value.shape != target[key].shape:
                if key not in (
                    "mlp_extractor.policy_net.0.weight",
                    "mlp_extractor.value_net.0.weight",
                ) or value.shape not in ((256, 88), (256, 117)):
                    raise ValueError("Unsupported architecture migration")
                target[key].zero_()
                target[key][:, : value.shape[1]] = value
            else:
                target[key] = value
        model.policy.load_state_dict(target)
    rng = np.random.default_rng(seed)
    torch.manual_seed(seed)
    xt, yt = torch.as_tensor(x), torch.as_tensor(y)
    modes = np.argmax(y[:, :7], axis=1)
    groups = [np.flatnonzero(modes == i) for i in range(7)] + [
        np.flatnonzero(y[np.arange(len(y)), 10 + 4 * modes] > 0),
        np.flatnonzero((np.abs(x[:, 0]) * 4.5 > 3.8) | (np.abs(x[:, 1]) * 3 > 2.3)),
        np.flatnonzero(np.min(x[:, [54, 58, 62]], axis=1) * 5 < 0.5),
        np.flatnonzero(np.abs(x[:, 121]) < 0.5),
    ]
    group_names = [f"mode_{i}" for i in range(7)] + [
        "release",
        "field_margin",
        "near_robot",
        "similar_ball_distance",
    ]
    sampling = {name: len(ids) for name, ids in zip(group_names, groups, strict=True)}
    groups = [g for g in groups if len(g)]
    opt = torch.optim.Adam(model.actor_parameters(), lr=learning_rate)
    weights = torch.zeros_like(yt)
    weights[:, :7] = 1
    for mode in range(7):
        ids = np.flatnonzero(modes == mode)
        weights[ids, 7 + 4 * mode : 11 + 4 * mode] = torch.tensor([15.0, 15.0, 5.0, 3.0])
    history = []
    for e in range(epochs):
        ids = np.concatenate([rng.choice(g, 2048) for g in groups])
        rng.shuffle(ids)
        losses = []
        for batch in np.array_split(ids, max(1, len(ids) // 512)):
            pred = model.policy.get_distribution(xt[batch]).distribution.mean
            loss = ((pred - yt[batch]).square() * weights[batch]).sum(1).mean() / 11
            opt.zero_grad()
            loss.backward()
            opt.step()
            losses.append(float(loss.detach()))
        history.append(float(np.mean(losses)))
        if (e + 1) % 10 == 0:
            print("Continuous fit", seed, e + 1, epochs, history[-1], flush=True)
    model.policy.optimizer.state.clear()
    result = save(
        model,
        out / f"seed{seed}" / "bootstrap.zip",
        {
            "stage": "supervised_initialization",
            "seed": seed,
            "epochs": epochs,
            "supervised_learning_rate": learning_rate,
            "observations": len(x),
            "data_sha256": digest(out / "data.npz"),
            "source": source,
            "source_sha256": digest(source) if source else None,
            "losses": history,
            "balanced_sampling_group_sizes": sampling,
        },
    )
    write(out / f"seed{seed}" / "fit.json", result)
    return model
