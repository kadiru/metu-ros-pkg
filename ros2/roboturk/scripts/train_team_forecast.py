"""Fit and qualify a plan-aware robot predictor on clean saved Webots matches."""

import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np
import torch
from torch import nn

from benchmark_motion_forecast import match_argument
from roboturk.team_forecast import HORIZONS, TEAMS, TeamModel, scene_features, team_features
from role_viewer import read_recording


WIDTH = 96
BATCH = 512
EPOCHS = 24
SEED = 42607


def read_matches(inputs):
    matches = []
    for name, folder in inputs:
        log, samples = read_recording(folder)
        if len(log["keys"]) != 10 or not any(f.get("allocation") for f in log["frames"]):
            raise ValueError(f"Expected an audited 5v5 match for {name}")
        scene = scene_features(log, samples)
        if not len(scene.origins):
            raise ValueError(f"No usable open play in {name}")
        future = scene.observed_future()
        plan = {team: team_features(log, samples, scene, team) for team in TEAMS}
        matches.append((name, log, samples, scene, future, plan))
    return matches


def fit(features, labels):
    rng = np.random.default_rng(SEED)
    torch.manual_seed(SEED)
    features = np.concatenate(features).astype(np.float32)
    labels = torch.from_numpy(np.concatenate(labels).astype(np.float32))
    mean = features.mean(axis=0)
    scale = features.std(axis=0).clip(0.05)
    inputs = torch.from_numpy((features - mean) / scale)
    model = nn.Sequential(
        nn.Linear(inputs.shape[1], WIDTH), nn.ReLU(), nn.Linear(WIDTH, WIDTH), nn.ReLU(),
        nn.Linear(WIDTH, labels.shape[1]),
    )
    optimizer = torch.optim.AdamW(model.parameters(), lr=0.002, weight_decay=0.001)
    for _ in range(EPOCHS):
        order = rng.permutation(len(inputs))
        for start in range(0, len(order), BATCH):
            batch = order[start : start + BATCH]
            optimizer.zero_grad()
            loss = nn.functional.mse_loss(model(inputs[batch]), labels[batch])
            loss.backward()
            optimizer.step()
    return TeamModel(
        mean, scale, tuple(
            (layer.weight.detach().numpy().copy(), layer.bias.detach().numpy().copy())
            for layer in model if isinstance(layer, nn.Linear)
        )
    )


def training_rows(matches):
    context, informed, target = [], [], []
    for _, _, _, scene, future, plan in matches:
        design = scene.context.reshape(-1, scene.context.shape[-1])
        labels = (
            (future - scene.positions[:, :, None, :]) * scene.direction[..., None, None]
        ).reshape(-1, 2 * len(HORIZONS))
        for team in TEAMS:
            # Both fits see the same scenes and labels in the same order.
            context.append(design)
            informed.append(np.column_stack((design, plan[team].reshape(len(design), -1))))
            target.append(labels)
    return context, informed, target


def summarize(errors):
    return {
        "cases": len(errors),
        "mean_200ms_m": float(errors[:, 0].mean()),
        "mean_600ms_m": float(errors[:, 2].mean()),
        "mean_1s_m": float(errors[:, -1].mean()),
        "mean_path_m": float(errors.mean()),
        "p95_1s_m": float(np.percentile(errors[:, -1], 95)),
    }


def match_info(match):
    name, log, _, scene, _, plan = match
    return dict(
        name=name, seed=log["seed"], origins=len(scene.origins),
        recorded_paths=sum(int(plan[team][..., 1].sum()) for team in TEAMS),
    )


def evaluate(matches, context_model, team_model):
    groups = defaultdict(list)
    per_match = []
    for match in matches:
        _, log, samples, scene, future, plan = match
        current = scene.standard_forecast()
        contextual = context_model.predict(scene.context, scene)
        errors = defaultdict(list)
        for team in TEAMS:
            known = np.array([key[0] == team for key in log["keys"]])
            trajectory = team_model.predict(np.concatenate((scene.context, plan[team]), axis=-1), scene)
            predictors = {"current": current, "context": contextual, "context_and_plans": trajectory}
            timed = plan[team][..., 1] == 1
            if np.any(timed):
                ablation = team_model.predict(
                    np.concatenate((scene.context, team_features(
                        log, samples, scene, team, use_timed_paths=False,
                    )), axis=-1), scene,
                )
                errors["timed_teammates", "commands_and_targets"].append(
                    np.linalg.norm(ablation - future, axis=-1)[timed]
                )
            for label, prediction in predictors.items():
                distances = np.linalg.norm(prediction - future, axis=-1)
                errors["teammates", label].append(distances[:, known].reshape(-1, len(HORIZONS)))
                errors["opponents", label].append(distances[:, ~known].reshape(-1, len(HORIZONS)))
                if np.any(timed):
                    errors["timed_teammates", label].append(distances[timed])
        per_match.append(match_info(match) | {"results": collate(errors)})
        for key, values in errors.items():
            groups[key].extend(values)
    return collate(groups), per_match


def collate(groups):
    out = defaultdict(dict)
    for (group, label), items in groups.items():
        out[group][label] = summarize(np.concatenate(items))
    return dict(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train", type=match_argument, nargs="+", required=True)
    parser.add_argument("--test", type=match_argument, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    torch.set_num_threads(1)
    train = read_matches(args.train)
    test = read_matches(args.test)
    if {match[1]["seed"] for match in train} & {match[1]["seed"] for match in test}:
        parser.error("Training and test seeds must be disjoint")
    motion, plan, target = training_rows(train)
    context_model, team_model = fit(motion, target), fit(plan, target)
    metrics, per_match = evaluate(test, context_model, team_model)
    args.output.mkdir(parents=True, exist_ok=True)
    team_model.save(args.output / "model.npz")
    context_model.save(args.output / "context-only.npz")
    report = {
        "sample_times_s": HORIZONS.tolist(),
        "fit": {"width": WIDTH, "epochs": EPOCHS, "batch": BATCH, "seed": SEED},
        "train": [match_info(match) for match in train],
        "test": per_match,
        "pooled": metrics,
    }
    (args.output / "results.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
