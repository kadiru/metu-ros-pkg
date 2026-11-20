"""Training-only boundary recovery and coordinate-preserving tactical PPO.

No action projection, boundary planner or choice override runs during inference.
"""

import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
import torch
from .decision_training import load, save
from .decision_play import DecisionEnv, evaluate
from .self_play import digest
from .self_play_training import actor_hash, write


def coordinate_hash(model):
    h = hashlib.sha256()
    for p in model.policy.mlp_extractor.policy_net.parameters():
        h.update(p.detach().cpu().numpy().tobytes())
    for p in (model.policy.action_net.weight, model.policy.action_net.bias):
        h.update(p[5:9].detach().cpu().numpy().tobytes())
    return h.hexdigest()


def preserve_coordinates(model):
    """Only option, owner, corner and target-update head rows may learn."""
    model.policy.mlp_extractor.policy_net.requires_grad_(False)
    for p in (model.policy.action_net.weight, model.policy.action_net.bias, model.policy.log_std):
        mask = torch.ones_like(p)
        mask[5:9] = 0
        p.register_hook(lambda grad, mask=mask: grad * mask)
        for value in model.policy.optimizer.state.get(p, {}).values():
            if isinstance(value, torch.Tensor) and value.shape == p.shape:
                value[5:9].zero_()
    return coordinate_hash(model)


def boundary_labels(observations, actions, cfg, margin=0.20):
    """Derived coaching labels on native observations, not measured action targets."""
    y = actions.copy()
    speed = observations[:, 15]
    if np.any(speed <= 0):
        raise ValueError("Invalid recorded defensive speed")
    for axis, scale, extent in ((0, 4.5, cfg.length / 2), (1, 3.0, cfg.width / 2)):
        q = observations[:, 33 + axis] * scale
        limit = extent - cfg.robot_radius - margin
        lower = np.clip(3 * (-limit - q) / speed, -1, 1)
        upper = np.clip(3 * (limit - q) / speed, -1, 1)
        y[:, axis] = np.clip(y[:, axis], lower, upper)
    return y


def repair_keeper(model, observations, cfg, seed, epochs=100):
    rng = np.random.default_rng(seed)
    x = np.asarray(observations, np.float32)
    original = np.concatenate(
        [model.predict(q, deterministic=True)[0] for q in np.array_split(x, 100)]
    )
    y = boundary_labels(x, original, cfg)
    changed = np.max(abs(y - original), axis=1) > 1e-6
    if not changed.any():
        raise ValueError("No boundary coaching samples")
    groups = [np.flatnonzero(changed), np.flatnonzero(~changed)]
    inputs, targets = torch.as_tensor(x), torch.as_tensor(y)
    optimizer = torch.optim.Adam(model.actor_parameters(), lr=1e-4)
    losses = []
    before = actor_hash(model)
    for epoch in range(epochs):
        ids = np.concatenate([rng.choice(g, 8192, replace=True) for g in groups if len(g)])
        rng.shuffle(ids)
        for batch in np.array_split(ids, len(ids) // 512):
            pred = model.policy.get_distribution(inputs[batch]).distribution.mean
            loss = ((pred - targets[batch]) ** 2).mean()
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            losses.append(float(loss.detach()))
        if (epoch + 1) % 20 == 0:
            print(
                f"Keeper boundary coaching {epoch + 1}/{epochs}: {np.mean(losses[-32:]):.7f}",
                flush=True,
            )
    model.policy.optimizer.state.clear()
    return {
        "method": "Supervised derived boundary labels on native observations; no runtime projection",
        "epochs": epochs,
        "seed": seed,
        "samples": len(x),
        "corrected_samples": int(changed.sum()),
        "margin_m": 0.20,
        "gain_per_second": 3.0,
        "learning_rate": 1e-4,
        "stale_ppo_optimizer_momenta_cleared": True,
        "actor_before": before,
        "actor_after": actor_hash(model),
        "last_mean_loss": float(np.mean(losses[-32:])),
    }


def prepare(backend, cfg, output, parent, seed=457):
    output, parent = Path(output), Path(parent)
    if (output / "bootstrap-bundle.json").exists():
        raise ValueError("Refusing to replace prepared initialization")
    league = json.loads((parent / "league.json").read_text())
    latest = league["snapshots"][-1]
    selected = dict(latest)
    selected["manager"] = json.loads((parent / "dagger-v3/bundle.json").read_text())["manager"]
    # Preserve the pre-PPO target function after observed geometric regression.
    models = load(selected, cfg)
    from .self_play_training import SelfPlayPPO

    models["keeper"] = SelfPlayPPO.load(selected["keeper"]["path"], device="cpu")
    output.mkdir(parents=True, exist_ok=True)
    for role in ("manager", "defender", "keeper"):
        shutil.copy2(parent / (role + "-data.npz"), output / (role + "-data.npz"))
    data = np.load(output / "keeper-data.npz")
    receipt = repair_keeper(models["keeper"], data["observations"], cfg, seed)
    receipt.update(
        parent_models=selected,
        discarded_manager_ppo=latest["manager"],
        source_data_sha256=digest(output / "keeper-data.npz"),
        rationale="Prior 3-generation league retained; target regression drift and keeper field exits persisted in development",
    )
    bundle = save(
        models,
        output / "prepared",
        cfg,
        {"stage": "boundary_coaching_and_preserved_manager", "seed": seed},
    )
    write(output / "preparation.json", receipt)
    write(output / "bootstrap-bundle.json", bundle)
    for name, base, count in [
        ("prepared-development", 3200000, 60),
        ("prepared-broad", 3201000, 120),
    ]:
        result = evaluate(DecisionEnv(backend, cfg, models), count, base)
        write(output / (name + ".json"), result)
        print(name, result["summary"], flush=True)
