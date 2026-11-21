"""Native preparation and historical self-play for the integrated skill selector."""

from pathlib import Path
import hashlib
import json
import gymnasium as gym
import numpy as np
import torch
from stable_baselines3.common.distributions import DiagGaussianDistribution
from stable_baselines3.common.policies import ActorCriticPolicy
from .strategy_options import StrategyMatch, OBSERVATIONS, ACTIONS, CONTRACT, MODES
from .continuous import summary
from .self_play_training import SelfPlayPPO, actor_hash


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, separators=(",", ":")) + "\n")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class ShapeEnv(gym.Env):
    observation_space = gym.spaces.Box(-np.inf, np.inf, (OBSERVATIONS,), np.float32)
    action_space = gym.spaces.Box(-1, 1, (ACTIONS,), np.float32)


class SelectorDistribution(DiagGaussianDistribution):
    def proba_distribution(self, mean_actions, log_std):
        super().proba_distribution(mean_actions, log_std)
        self.choices = torch.distributions.Categorical(logits=mean_actions[:, :10] * 3)
        return self

    def chosen(self, indices):
        actions = self.distribution.mean.clone()
        actions[:, :10] = -0.8
        actions[torch.arange(len(actions), device=actions.device), indices] = 0.8
        return actions

    def sample(self):
        return self.chosen(self.choices.sample())

    def mode(self):
        return self.chosen(self.choices.logits.argmax(1))

    def log_prob(self, actions):
        return self.choices.log_prob(actions[:, :10].argmax(1))

    def entropy(self):
        return self.choices.entropy()


class SelectorPolicy(ActorCriticPolicy):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.action_dist = SelectorDistribution(ACTIONS)


def save(model, path, metadata):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    model.save(path)
    write(
        path.with_suffix(".json"),
        {
            **metadata,
            "contract": CONTRACT,
            "actor_sha256": actor_hash(model),
            "checkpoint_sha256": digest(path),
        },
    )


def load(path):
    path = Path(path)
    meta = json.loads(path.with_suffix(".json").read_text())
    assert meta["contract"] == CONTRACT and meta["checkpoint_sha256"] == digest(path)
    model = SelfPlayPPO.load(path, device="cpu")
    assert model.observation_space.shape == (OBSERVATIONS,) and model.action_space.shape == (
        ACTIONS,
    )
    assert actor_hash(model) == meta["actor_sha256"]
    return model


def evaluate(backend, bank, models, count, seed, duration=20, trace=True, recorder=None):
    match = StrategyMatch(backend, bank, duration, trace)
    records = []
    for i in range(count):
        match.reset(seed + i)
        if recorder:
            recorder.begin(i, match)
        while not match.finished:
            match.restart_if_needed()
            if models is None:
                from .strategy_teacher import actions

                a = actions(match)
            else:
                obs = match.observations()
                a = np.concatenate(
                    [
                        models[0].predict(obs[:2], deterministic=True)[0],
                        models[1].predict(obs[2:], deterministic=True)[0],
                    ]
                )
            match.step(a)
            if recorder:
                recorder.step(match)
        records.append(match.result())
        print("Strategy evaluation", i + 1, count, summarize(records), flush=True)
    return {
        "contract": CONTRACT,
        "control": "training_selector_teacher" if models is None else "neural",
        "backend": backend.name,
        "motors": bank.descriptors,
        "duration": duration,
        "matches": records,
        "summary": summarize(records),
    }


def summarize(records):
    result = summary(records)
    result["skill_outcomes"] = {
        kind: sum(e["type"] == kind for r in records for e in r["option_events"])
        for kind in ("receive_stop", "redirect_kick", "redirect_goal", "clearance_complete")
    }
    result["new_skill_ticks"] = {
        kind: sum(
            v for r in records for k, v in r["strategy_ticks"].items() if k.endswith("/" + kind)
        )
        for kind in MODES[7:]
    }
    return result


def collect(backend, bank, out, count, seed, duration=20, model=None, mix=0):
    from .strategy_teacher import actions

    out = Path(out)
    if out.exists():
        raise ValueError("Retain existing collection")
    out.mkdir(parents=True)
    match = StrategyMatch(backend, bank, duration)
    rng = np.random.default_rng(seed)
    xs, ys, records = [], [], []
    for i in range(count):
        match.reset(seed + i)
        while not match.finished:
            match.restart_if_needed()
            x = match.observations()
            y = actions(match)
            if match.steps % 2 == 0:
                xs.extend(x)
                ys.extend(y)
            a = y.copy()
            if model is not None:
                neural = model.predict(x, deterministic=True)[0]
                chosen = rng.random(4) < mix
                a[chosen] = neural[chosen]
            match.step(a)
        records.append(match.result())
        if (i + 1) % 4 == 0:
            print("Strategy collect", i + 1, count, summarize(records), flush=True)
    np.savez_compressed(
        out / "data.npz",
        observations=np.asarray(xs, np.float32),
        actions=np.asarray(ys, np.float32),
    )
    write(
        out / "collection.json",
        {
            "seed": seed,
            "count": count,
            "mix": mix,
            "backend": backend.name,
            "matches": records,
            "data_sha256": digest(out / "data.npz"),
            "motors": bank.descriptors,
        },
    )


def fit(data_paths, out, seed=811, epochs=120, parent=None):
    torch.set_num_threads(1)
    torch.manual_seed(seed)
    xs, ys = [], []
    for path in data_paths:
        data = np.load(path)
        xs.append(data["observations"])
        ys.append(data["actions"])
    x, y = np.concatenate(xs), np.concatenate(ys)
    model = (
        load(parent)
        if parent
        else SelfPlayPPO(
            SelectorPolicy,
            ShapeEnv(),
            seed=seed,
            device="cpu",
            n_steps=1024,
            batch_size=256,
            n_epochs=5,
            learning_rate=3e-5,
            gamma=0.999,
            clip_range=0.1,
            policy_kwargs={"net_arch": {"pi": [256, 256], "vf": [256, 256]}, "log_std_init": -5},
        )
    )
    modes = y[:, :10].argmax(1)
    groups = [np.flatnonzero(modes == i) for i in range(10)]
    groups = [g for g in groups if len(g)]
    inputs, labels = torch.as_tensor(x), torch.as_tensor(y)
    rng = np.random.default_rng(seed)
    opt = torch.optim.Adam(model.actor_parameters(), lr=3e-4)
    for epoch in range(epochs):
        indices = np.concatenate([rng.choice(g, 2048, replace=True) for g in groups])
        rng.shuffle(indices)
        losses = []
        for ids in np.array_split(indices, max(1, len(indices) // 512)):
            pred = model.policy.get_distribution(inputs[ids]).distribution.mean
            target = labels[ids, :10].argmax(1)
            loss = torch.nn.functional.cross_entropy(pred[:, :10] * 3, target)
            special = target >= 7
            if special.any():
                loss = loss + 30 * ((pred[special, 10:] - labels[ids][special, 10:]) ** 2).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
            losses.append(loss.item())
        if (epoch + 1) % 20 == 0:
            print("Strategy fit", seed, epoch + 1, epochs, float(np.mean(losses)), flush=True)
    model.policy.optimizer.state.clear()
    save(
        model,
        out,
        {
            "seed": seed,
            "method": "native strategic demonstrations/DAgger",
            "epochs": epochs,
            "datasets": [{"path": str(p), "sha256": digest(p)} for p in data_paths],
            "counts": {str(i): int((modes == i).sum()) for i in range(10)},
        },
    )
    return model
