"""Explicit v2-to-v3 learned receiving-target memory preparation."""

import json
from pathlib import Path
import shutil
import numpy as np
import torch
from .decision_play import DecisionEnv, evaluate
from .decision_training import fit, save, collect, new_manager
from .self_play import load_model, digest
from .self_play_training import SelfPlayPPO, actor_hash, write


def extend_labels(x, y):
    """Gate labels from teacher target changes, never from a runtime planner."""
    if y.shape[1] != 10 or x.shape[1] != 75:
        raise ValueError("Expected v2 observations/actions")
    reset = np.max(np.abs(x[:, 46:56]), axis=1) < 1e-7
    change = np.r_[True, np.max(np.abs(np.diff(y[:, 5:7], axis=0)), axis=1) > 1e-6]
    gate = np.where(reset | change, 0.8, -0.8).astype(np.float32)
    return np.column_stack((y, gate))


def migrate(old, seed):
    model = new_manager(seed)
    old_state, state = old.policy.state_dict(), model.policy.state_dict()
    with torch.no_grad():
        for name, target in state.items():
            source = old_state[name]
            if target.shape == source.shape:
                target.copy_(source)
            elif name in ("action_net.weight", "action_net.bias", "log_std"):
                target[:10].copy_(source)
                target[10].fill_(
                    -4.5 if name == "log_std" else -0.8 if name.endswith("bias") else 0
                )
            else:
                raise ValueError("Unrecognized migration shape: " + name)
    model.policy.load_state_dict(state)
    return model


def prepare(backend, cfg, output, seed=447):
    output = Path(output)
    if (output / "league.json").exists() or (output / "prepared-v3").exists():
        raise ValueError("Refusing to overwrite existing league/preparation")
    initial = json.loads((output / "bootstrap-bundle.json").read_text())
    write(output / "bootstrap-v2-bundle.json", initial)
    models = {}
    for role, entry in initial.items():
        if digest(entry["path"]) != entry["sha256"]:
            raise ValueError("Changed v2 parent")
        models[role] = (
            migrate(SelfPlayPPO.load(entry["path"], device="cpu"), seed)
            if role == "manager"
            else load_model(entry["path"], role, cfg)
        )
    for role in ("manager", "defender", "keeper"):
        target = output / "v2-data" / (role + "-data.npz")
        target.parent.mkdir(exist_ok=True)
        shutil.copy2(output / (role + "-data.npz"), target)
    data = np.load(output / "manager-data.npz")
    x, y = data["observations"], extend_labels(data["observations"], data["actions"])
    before = actor_hash(models["manager"])
    receipt = fit(models["manager"], x, y, 120, seed)
    receipt.update(
        parent=initial,
        actor_before=before,
        actor_after=actor_hash(models["manager"]),
        source_data_sha256=digest(output / "manager-data.npz"),
        change="v3 eleven outputs: learned target update/retain gate; v2 labels extended from consecutive teacher target changes and reset rows",
    )
    bundle = save(
        models, output / "prepared-v3", cfg, {"stage": "v3_commitment_preparation", "seed": seed}
    )
    write(output / "prepared-v3/training.json", receipt)
    result = evaluate(DecisionEnv(backend, cfg, models), 60, 3200000)
    write(output / "prepared-v3-development.json", result)
    print("Prepared v3", result["summary"], flush=True)
    data, result = collect(backend, cfg, models, 150, 3040000, 0.8)
    write(output / "dagger-v3-collection.json", result)
    print("v3 DAgger collection", result["summary"], flush=True)
    for role, (nx, ny) in data.items():
        old = np.load(output / "v2-data" / (role + "-data.npz"))
        ox, oy = old["observations"], old["actions"]
        if role == "manager":
            oy = extend_labels(ox, oy)
        x, y = np.concatenate([ox, nx]), np.concatenate([oy, ny])
        np.savez_compressed(output / (role + "-data.npz"), observations=x, actions=y)
        if role == "manager":
            receipt = fit(models[role], x, y, 120, seed + 1)
            write(output / "dagger-v3-fit.json", receipt)
    bundle = save(
        models,
        output / "dagger-v3",
        cfg,
        {"stage": "v3_native_dagger", "seed": seed + 1, "parent": bundle},
    )
    result = evaluate(DecisionEnv(backend, cfg, models), 60, 3200000)
    write(output / "dagger-v3-development.json", result)
    write(output / "bootstrap-bundle.json", bundle)
    print("v3 neural development", result["summary"], flush=True)
