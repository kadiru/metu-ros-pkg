"""Explicit v3-to-v4 flank/magnitude factorization; no analytic inference target."""

import json
from pathlib import Path
import shutil
import numpy as np
import torch
from .decision_play import DecisionEnv, evaluate
from .decision_training import new_manager, save, fit
from .self_play import digest, load_model
from .self_play_training import SelfPlayPPO, actor_hash, write


def extend_labels(actions):
    if actions.shape[1] != 11:
        raise ValueError("Expected v3 action labels")
    y = np.column_stack((actions, np.where(actions[:, 6] >= 0, 0.8, -0.8))).astype(np.float32)
    y[:, 6] = abs(y[:, 6])
    return y


def migrate(old, seed):
    model = new_manager(seed)
    original, state = old.policy.state_dict(), model.policy.state_dict()
    with torch.no_grad():
        for name, target in state.items():
            source = original[name]
            if target.shape == source.shape:
                target.copy_(source)
            elif name in ("action_net.weight", "action_net.bias", "log_std"):
                target[:11].copy_(source)
                target[11].copy_(source[6])
            else:
                raise ValueError("Unrecognized migration shape: " + name)
    model.policy.load_state_dict(state)
    return model


def prepare(backend, cfg, output, seed=477):
    output = Path(output)
    if (output / "league.json").exists() or (output / "flank-prepared").exists():
        raise ValueError("Refusing to replace existing flank preparation/league")
    initial = json.loads((output / "bootstrap-bundle.json").read_text())
    write(output / "bootstrap-v3-bundle.json", initial)
    models = {}
    for role, entry in initial.items():
        if digest(entry["path"]) != entry["sha256"]:
            raise ValueError("Changed v3 parent")
        models[role] = (
            migrate(SelfPlayPPO.load(entry["path"], device="cpu"), seed)
            if role == "manager"
            else load_model(entry["path"], role, cfg)
        )
    source = output / "manager-data.npz"
    shutil.copy2(source, output / "manager-v3-data.npz")
    data = np.load(source)
    x, y = data["observations"], extend_labels(data["actions"])
    before = actor_hash(models["manager"])
    receipt = fit(models["manager"], x, y, 120, seed)
    receipt.update(
        parent_models=initial,
        actor_before=before,
        actor_after=actor_hash(models["manager"]),
        source_data_sha256=digest(source),
        conversion="75 inputs unchanged; action6 absolute lateral magnitude, new action11 learned flank score. Signed executed coordinate remains in observation memory.",
    )
    np.savez_compressed(source, observations=x, actions=y)
    bundle = save(
        models,
        output / "flank-prepared",
        cfg,
        {"stage": "v4_factored_flank_preparation", "seed": seed},
    )
    write(output / "flank-preparation.json", receipt)
    write(output / "bootstrap-bundle.json", bundle)
    for name, base, count in [("flank-development", 3200000, 60), ("flank-broad", 3201000, 120)]:
        result = evaluate(DecisionEnv(backend, cfg, models), count, base)
        write(output / (name + ".json"), result)
        print(name, result["summary"], flush=True)
