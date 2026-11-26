"""Explicit v1-to-v2 decision preparation; no reserved data or hidden inference teacher."""

import json
from pathlib import Path
import shutil
import numpy as np
from .decision_play import DecisionEnv, evaluate
from .decision_training import fit, save, collect
from .self_play import load_model, digest
from .self_play_training import SelfPlayPPO, actor_hash, write


def prepare(backend, cfg, output, seed=437):
    output = Path(output)
    if (output / "league.json").exists() or (output / "prepared-v2").exists():
        raise ValueError("Refusing to overwrite an existing league/preparation")
    initial = json.loads((output / "bootstrap-bundle.json").read_text())
    write(output / "bootstrap-v1-bundle.json", initial)
    models = {}
    for role, entry in initial.items():
        if digest(entry["path"]) != entry["sha256"]:
            raise ValueError("Changed v1 parent")
        models[role] = (
            SelfPlayPPO.load(entry["path"], device="cpu")
            if role == "manager"
            else load_model(entry["path"], role, cfg)
        )
    for role in ("manager", "defender", "keeper"):
        target = output / "v1-data" / (role + "-data.npz")
        target.parent.mkdir(exist_ok=True)
        shutil.copy2(output / (role + "-data.npz"), target)
    data = np.load(output / "manager-data.npz")
    before = actor_hash(models["manager"])
    receipt = fit(models["manager"], data["observations"], data["actions"], 120, seed)
    receipt.update(
        parent=initial,
        actor_before=before,
        actor_after=actor_hash(models["manager"]),
        change="v2 learned discrete corner, executed-action memory; reset/switch-balanced fitting of original native v1 observations",
    )
    bundle = save(
        models, output / "prepared-v2", cfg, {"stage": "v2_balanced_preparation", "seed": seed}
    )
    write(output / "prepared-v2/training.json", receipt)
    result = evaluate(DecisionEnv(backend, cfg, models), 60, 3200000)
    write(output / "prepared-v2-development.json", result)
    print("Prepared v2", result["summary"], flush=True)
    data, result = collect(backend, cfg, models, 150, 3030000, 0.8)
    write(output / "dagger-v2-collection.json", result)
    print("v2 DAgger collection", result["summary"], flush=True)
    for role, (x, y) in data.items():
        old = np.load(output / "v1-data" / (role + "-data.npz"))
        x = np.concatenate([old["observations"], x])
        y = np.concatenate([old["actions"], y])
        np.savez_compressed(output / (role + "-data.npz"), observations=x, actions=y)
        if role == "manager":
            receipt = fit(models[role], x, y, 120, seed + 1)
            write(output / "dagger-v2-fit.json", receipt)
    bundle = save(
        models,
        output / "dagger-v2",
        cfg,
        {"stage": "v2_native_dagger", "seed": seed + 1, "parent": bundle},
    )
    result = evaluate(DecisionEnv(backend, cfg, models), 60, 3200000)
    write(output / "dagger-v2-development.json", result)
    write(output / "bootstrap-bundle.json", bundle)
    print("v2 neural development", result["summary"], flush=True)
