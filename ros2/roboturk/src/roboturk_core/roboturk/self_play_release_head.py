"""Train only neural kick readouts, preserving all learned movement exactly."""

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


def labels(role, x, reference, cfg, shot_range=1.70):
    if role == "transfer":
        active = x[:, 31] < 0.5
        unsafe = active & ((x[:, 32] < 0.999) | (x[:, 11] < 0.5) | (x[:, 16] * 5 < 1.05))
        fire = active & ~unsafe & (reference[:, 3] > 0)
        return fire, unsafe
    if role != "finish":
        raise ValueError("Unknown role")
    goal = x[:, 12:14] * 5
    corner = x[:, 21:23] * 3
    tangent = corner - goal
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-6)
    normal = np.column_stack((tangent[:, 1], -tangent[:, 0]))
    normal *= np.where((normal * goal).sum(axis=1) >= 0, 1, -1)[:, None]
    distance = (normal * goal).sum(axis=1) / np.maximum(normal[:, 0], 0.1)
    cross = (np.column_stack((distance, np.zeros(len(x)))) - goal) * tangent
    legal_ray = (normal[:, 0] > 0.1) & (
        np.abs(cross.sum(axis=1)) < cfg.goal_width / 2 - cfg.ball_radius - 0.025
    )
    held = x[:, 11] > 0.5
    in_range = x[:, 16] * 5 <= shot_range
    fire = held & in_range & legal_ray & (np.abs(x[:, 8] * cfg.max_omega) < 0.5)
    # Preserve the already-fired request while a free shot travels, as in the
    # existing teacher. The actuator's held-edge rule prevents another kick.
    travelling = (
        ~held
        & (np.linalg.norm(x[:, 4:6], axis=1) * skill_limits(cfg).max_speed > 1.5)
        & (x[:, 16] * 5 < 2)
    )
    return fire | travelling, held & ~fire


def latent(model, x):
    with torch.no_grad():
        values = []
        for batch in np.array_split(x, max(1, (len(x) + 2047) // 2048)):
            features = model.policy.extract_features(torch.as_tensor(batch))
            values.append(model.policy.mlp_extractor.forward_actor(features))
        return torch.cat(values)


def prepare(
    source,
    output,
    epochs=80,
    seed=357,
    negative_weight=1,
    shot_range=1.70,
    dwell_augmentation=False,
):
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
        "method": "frozen-motion neural kick-readout training",
        "reserved_observations_used": False,
        "negative_weight": negative_weight,
        "shot_range": shot_range,
        "dwell_augmentation": dwell_augmentation,
        "roles": {},
    }
    for i, role in enumerate(("transfer", "finish")):
        model = models[role]
        path = source / (role + "-data.npz")
        x = np.load(path)["observations"]
        original_count = len(x)
        reference = means(model, x)
        synthetic_count = 0
        if role == "transfer" and dwell_augmentation:
            # Counterfactual training states only: request a pass before the
            # possession dwell is complete, with every other feature retained.
            eligible = (reference[:, 3] > 0) & (x[:, 31] < 0.5)
            copies = []
            for dwell in (0.0, 0.25, 0.5, 0.75, 0.95):
                changed = x[eligible].copy()
                changed[:, 32] = dwell
                copies.append(changed)
            extra = np.concatenate(copies)
            synthetic_count = len(extra)
            x = np.concatenate((x, extra))
            reference = means(model, x)
        fire, unsafe = labels(role, x, reference, cfg, shot_range)
        groups = [np.arange(len(x)), np.flatnonzero(fire), np.flatnonzero(unsafe)]
        if any(len(g) < 100 for g in groups):
            raise ValueError("Insufficient release training coverage")
        early = (
            fire & (x[:, 27] < 0.5) & (x[:, 11] > 0.5)
            if role == "finish"
            else np.zeros(len(x), bool)
        )
        if role == "finish" and early.sum() < 100:
            raise ValueError("No meaningful early-phase firing examples")
        z = latent(model, x)
        head = torch.nn.Linear(z.shape[1], 1)
        with torch.no_grad():
            head.weight.copy_(model.policy.action_net.weight[3:4])
            head.bias.copy_(model.policy.action_net.bias[3:4])
        initial = [p.detach().clone() for p in head.parameters()]
        target = torch.as_tensor(fire.astype(np.float32) * 2 - 1)
        optimizer = torch.optim.Adam(head.parameters(), lr=3e-4)
        rng, losses = np.random.default_rng(seed + i), []
        before = actor_hash(model)
        for epoch in range(epochs):
            ids = np.concatenate([rng.choice(g, 8192) for g in groups])
            rng.shuffle(ids)
            values = []
            for batch in np.array_split(ids, 48):
                score = head(z[batch]).squeeze(-1)
                if role == "transfer" and dwell_augmentation:
                    desired = torch.as_tensor(reference[batch, 3].copy())
                    desired[torch.as_tensor(unsafe[batch])] = -1
                    loss = (score - desired).square().mean()
                else:
                    weight = torch.where(target[batch] > 0, 1.0, float(negative_weight))
                    loss = (weight * torch.nn.functional.softplus(-target[batch] * score)).mean()
                loss += 1e-3 * sum(
                    (p - original).square().mean()
                    for p, original in zip(head.parameters(), initial, strict=True)
                )
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                values.append(float(loss.detach()))
            losses.append(float(np.mean(values)))
            if (epoch + 1) % 10 == 0:
                print(f"Kick readout {role} {epoch + 1}/{epochs}: {losses[-1]:.6f}", flush=True)
        with torch.no_grad():
            model.policy.action_net.weight[3:4].copy_(head.weight)
            model.policy.action_net.bias[3:4].copy_(head.bias)
        model.policy.optimizer.state.clear()
        after = means(model, x)
        movement = [j for j in range(reference.shape[1]) if j != 3]
        if not np.array_equal(reference[:, movement], after[:, movement]):
            raise ValueError("Movement changed during kick-readout training")
        report["roles"][role] = {
            "actor_before": before,
            "actor_after": actor_hash(model),
            "dataset_sha256": digest(path),
            "observations": len(x),
            "native_observations": original_count,
            "synthetic_dwell_observations": synthetic_count,
            "positive_examples": int(fire.sum()),
            "early_phase_examples": int(early.sum()),
            "movement_outputs_bit_identical": True,
            "epoch_losses": losses,
            "false_positive_rate": float(((after[:, 3] > 0) & ~fire).sum() / max(1, (~fire).sum())),
            "false_negative_rate": float(((after[:, 3] <= 0) & fire).sum() / max(1, fire.sum())),
        }
    report["bundle"] = save_bundle(
        models, output, cfg, {"stage": "kick_readout_preparation", "seed": seed}
    )
    write(output / "training.json", report)
    print(
        json.dumps(
            {
                r: {k: v for k, v in d.items() if k != "epoch_losses"}
                for r, d in report["roles"].items()
            },
            indent=2,
        ),
        flush=True,
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--epochs", type=int, default=80)
    p.add_argument("--seed", type=int, default=357)
    p.add_argument("--negative-weight", type=float, default=1)
    p.add_argument("--shot-range", type=float, default=1.70)
    p.add_argument("--dwell-augmentation", action="store_true")
    a = p.parse_args()
    prepare(
        a.source.resolve(),
        a.output.resolve(),
        a.epochs,
        a.seed,
        a.negative_weight,
        a.shot_range,
        a.dwell_augmentation,
    )


if __name__ == "__main__":
    main()
