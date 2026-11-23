"""Training-only pass-dwell and earlier in-range shooting distillation."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import numpy as np
import torch
from .model import Config, skill_limits
from .self_play import ROLES, digest
from .self_play_repair import means
from .self_play_training import SelfPlayPPO, actor_hash, save_bundle, write


def timing_labels(role, x, reference, cfg):
    labels = reference.copy()
    if role == "transfer":
        active = x[:, 31] < 0.5
        unsafe = active & ((x[:, 32] < 0.999) | (x[:, 11] < 0.5) | (x[:, 16] * 5 < 1.05))
        firing = active & ~unsafe & (reference[:, 3] > 0)
        labels[unsafe, 3] = -1
        labels[firing, 3] = 1
        return labels, unsafe, firing
    if role != "finish":
        raise ValueError("Unknown attacking role")
    # In this receiving curriculum, an in-range retained ball has already travelled
    # well beyond the minimum0.5m carry. The independent task monitor still checks it.
    near = (x[:, 11] > 0.5) & (x[:, 16] * 5 <= 1.68)
    angle = x[:, 23]
    omega = np.clip(16 * angle, -3, 3)
    labels[near, 0] = 3 * (x[near, 0] * 3 - cfg.possession_x) / skill_limits(cfg).max_speed
    labels[near, 1] = (-cfg.possession_x * omega[near] + 3 * x[near, 1] * 3) / skill_limits(
        cfg
    ).max_speed
    labels[near, 2] = omega[near] / cfg.max_omega
    speed = np.linalg.norm(x[:, 6:8], axis=1) * skill_limits(cfg).max_speed
    firing = (
        near & (np.abs(angle) < 0.015) & (speed < 0.18) & (np.abs(x[:, 8] * cfg.max_omega) < 0.3)
    )
    labels[near, 3] = -1
    labels[firing, :3] = 0
    labels[firing, 3] = 1
    return labels, near, firing


def prepare(source, output, epochs=80, seed=347):
    torch.set_num_threads(1)
    torch.manual_seed(seed)
    output.mkdir(parents=True, exist_ok=False)
    parent = json.loads((source / "league.json").read_text())["snapshots"][-1]
    for entry in parent.values():
        if digest(entry["path"]) != entry["sha256"]:
            raise ValueError("Parent checkpoint changed")
    cfg = Config(
        **json.loads(Path(parent["transfer"]["path"]).with_suffix(".json").read_text())["config"]
    )
    models = {r: SelfPlayPPO.load(parent[r]["path"], device="cpu") for r in ROLES}
    report = {
        "parent": parent,
        "seed": seed,
        "epochs": epochs,
        "config": asdict(cfg),
        "method": "training-only stable-pass and earlier in-range shot distillation",
        "reserved_observations_used": False,
        "roles": {},
    }
    for i, role in enumerate(("transfer", "finish")):
        model = models[role]
        data_path = source / (role + "-data.npz")
        x = np.load(data_path)["observations"]
        reference = means(model, x)
        y, focus, firing = timing_labels(role, x, reference, cfg)
        groups = [np.arange(len(x)), np.flatnonzero(focus), np.flatnonzero(firing)]
        if any(len(g) < 50 for g in groups):
            raise ValueError("Insufficient training coverage for release calibration")
        before = actor_hash(model)
        rng = np.random.default_rng(seed + i)
        inputs, targets = torch.as_tensor(x), torch.as_tensor(y)
        optimizer = torch.optim.Adam(model.actor_parameters(), lr=1e-4)
        motion = [j for j in range(y.shape[1]) if j != 3]
        losses = []
        for epoch in range(epochs):
            ids = np.concatenate([rng.choice(g, 8192) for g in groups])
            rng.shuffle(ids)
            values = []
            for batch in np.array_split(ids, 48):
                prediction = model.policy.get_distribution(inputs[batch]).distribution.mean
                delta = prediction - targets[batch]
                loss = (20 if role == "transfer" else 1) * delta[:, motion].square().mean() + delta[
                    :, 3
                ].square().mean()
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                values.append(float(loss.detach()))
            losses.append(float(np.mean(values)))
            if (epoch + 1) % 10 == 0:
                print(f"Release timing {role} {epoch + 1}/{epochs}: {losses[-1]:.7f}", flush=True)
        model.policy.optimizer.state.clear()
        after = means(model, x)
        report["roles"][role] = {
            "actor_before": before,
            "actor_after": actor_hash(model),
            "dataset_sha256": digest(data_path),
            "observations": len(x),
            "focus_observations": int(focus.sum()),
            "firing_observations": int(firing.sum()),
            "epoch_losses": losses,
            "ppo_optimizer_state_reset": True,
            "focus_mse_before": float(((reference[focus] - y[focus]) ** 2).mean()),
            "focus_mse_after": float(((after[focus] - y[focus]) ** 2).mean()),
            "other_reference_mse": float(((after[~focus] - reference[~focus]) ** 2).mean()),
            "unsafe_positive_rate_before": float((reference[focus, 3] > 0).mean())
            if role == "transfer"
            else None,
            "unsafe_positive_rate_after": float((after[focus, 3] > 0).mean())
            if role == "transfer"
            else None,
        }
    report["bundle"] = save_bundle(
        models, output, cfg, {"stage": "release_timing_preparation", "seed": seed}
    )
    write(output / "training.json", report)
    print(json.dumps(report["roles"], indent=2), flush=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--epochs", type=int, default=80)
    p.add_argument("--seed", type=int, default=347)
    a = p.parse_args()
    prepare(a.source.resolve(), a.output.resolve(), a.epochs, a.seed)


if __name__ == "__main__":
    main()
