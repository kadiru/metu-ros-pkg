"""Training-only close-spacing distillation on retained native training observations.

No projection from this module is used by a deployed controller.
"""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import numpy as np
import torch
from .model import Config
from .self_play import ROLES, digest
from .self_play_training import SelfPlayPPO, actor_hash, save_bundle, write


def spacing_labels(x, teacher, reference, max_speed=2.0):
    """Conservative relative-velocity labels in the existing attack-frame contract."""
    sine, cosine = x[:, 6], x[:, 7]
    neighbors = []
    for start in (35, 39):
        bx, by = x[:, start] * 3, x[:, start + 1] * 3
        vx, vy = x[:, start + 2] * max_speed, x[:, start + 3] * max_speed
        neighbors.append(
            (
                np.column_stack((cosine * bx - sine * by, sine * bx + cosine * by)),
                np.column_stack((cosine * vx - sine * vy, sine * vx + cosine * vy)),
            )
        )
    # The old45-input contract lacks the other defender's velocity. A stationary
    # forecast is conservative when choosing whether to approach that teammate.
    neighbors.append((x[:, 43:45] * 3, np.zeros((len(x), 2), np.float32)))
    distances = [np.linalg.norm(p, axis=1) for p, _ in neighbors]
    nearest = np.min(distances, axis=0)
    velocity = teacher[:, :2] * x[:, 15:16]
    for _ in range(3):
        for (position, other_velocity), distance in zip(neighbors, distances, strict=True):
            normal = position / np.maximum(distance[:, None], 1e-6)
            closing = ((velocity - other_velocity) * normal).sum(axis=1)
            allowed = 2 * np.maximum(0, distance - 0.28)
            correction = np.where(distance < 0.65, np.maximum(0, closing - allowed), 0)
            velocity -= correction[:, None] * normal
    speed = x[:, 15:16]
    velocity /= np.maximum(1, np.linalg.norm(velocity, axis=1, keepdims=True) / speed)
    labels = reference.copy()
    close = nearest < 0.65
    labels[close, :2] = velocity[close] / speed[close]
    return labels.astype(np.float32), nearest


def means(model, x):
    with torch.no_grad():
        return np.concatenate(
            [
                model.policy.get_distribution(torch.as_tensor(batch)).distribution.mean.numpy()
                for batch in np.array_split(x, max(1, (len(x) + 2047) // 2048))
            ]
        )


def repair(source, output, epochs=80, seed=337):
    torch.set_num_threads(1)
    torch.manual_seed(seed)
    output.mkdir(parents=True, exist_ok=False)
    league = json.loads((source / "league.json").read_text())
    parent = league["snapshots"][-1]
    cfg = Config(
        **json.loads(Path(parent["defender"]["path"]).with_suffix(".json").read_text())["config"]
    )
    models = {r: SelfPlayPPO.load(parent[r]["path"], device="cpu") for r in ROLES}
    report = {
        "parent": parent,
        "seed": seed,
        "epochs": epochs,
        "config": asdict(cfg),
        "method": "training-only close-spacing relabeling and weighted actor distillation",
        "reserved_observations_used": False,
        "roles": {},
    }
    for i, role in enumerate(("defender", "keeper")):
        model = models[role]
        data_path = source / (role + "-data.npz")
        data = np.load(data_path)
        x, teacher = data["observations"], data["actions"]
        reference = means(model, x)
        y, nearest = spacing_labels(x, teacher, reference, cfg.max_speed)
        near = np.flatnonzero(nearest < 0.65)
        if len(near) < 100:
            raise ValueError("Insufficient close-range training observations")
        before = actor_hash(model)
        rng = np.random.default_rng(seed + i)
        inputs, labels = torch.as_tensor(x), torch.as_tensor(y)
        optimizer = torch.optim.Adam(model.actor_parameters(), lr=1e-4)
        losses = []
        for epoch in range(epochs):
            ids = np.concatenate([rng.integers(0, len(x), 8192), rng.choice(near, 32768)])
            rng.shuffle(ids)
            batch_losses = []
            for batch in np.array_split(ids, 80):
                pred = model.policy.get_distribution(inputs[batch]).distribution.mean
                loss = ((pred - labels[batch]) ** 2).mean()
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                batch_losses.append(float(loss.detach()))
            losses.append(float(np.mean(batch_losses)))
            if (epoch + 1) % 10 == 0:
                print(f"Spacing repair {role} {epoch + 1}/{epochs}: {losses[-1]:.7f}", flush=True)
        after = means(model, x)
        report["roles"][role] = {
            "actor_before": before,
            "actor_after": actor_hash(model),
            "observations": len(x),
            "near_observations": len(near),
            "dataset_sha256": digest(data_path),
            "epoch_losses": losses,
            "near_label_mse_before": float(((reference[near] - y[near]) ** 2).mean()),
            "near_label_mse_after": float(((after[near] - y[near]) ** 2).mean()),
            "far_reference_mse": float(
                ((after[nearest >= 0.65] - reference[nearest >= 0.65]) ** 2).mean()
            ),
        }
    bundle = save_bundle(models, output, cfg, {"stage": "close_spacing_preparation", "seed": seed})
    report["bundle"] = bundle
    write(output / "training.json", report)
    print(json.dumps(report["roles"], indent=2), flush=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--epochs", type=int, default=80)
    p.add_argument("--seed", type=int, default=337)
    a = p.parse_args()
    repair(a.source.resolve(), a.output.resolve(), a.epochs, a.seed)


if __name__ == "__main__":
    main()
