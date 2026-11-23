"""Fit a short-range receiving head from labeled native Webots rollouts."""

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from .capture import CaptureTeacher
from .model import Intent, Skill, skill_limits
from .team import TeamCheckpoint
from .team_contract import RECEIVER_CONTRACT, RECEIVER_RANGE, TEAM_CONTRACT, receiver_features


def fit_receiver(env, checkpoint, output, episodes, seed):
    checkpoint, output = Path(checkpoint), Path(output)
    policy = TeamCheckpoint(checkpoint, env.cfg)
    if policy.contract != TEAM_CONTRACT or env.backend.name != "webots_ideal_body_velocity":
        raise ValueError("Fit from a v2 transfer policy in native Webots")
    limits = skill_limits(env.cfg)
    rng = np.random.default_rng(seed)
    teacher = CaptureTeacher()
    features, labels, records = [], [], []
    for index in range(episodes):
        env.reset(seed=seed + index)
        samples, actions = [], []
        while True:
            action = policy.action(env)
            r = env.world.robot(env.receiver)
            x = receiver_features(env.world, env.receiver, env.cfg) if env.state.passed else None
            if x is not None:
                cmd = teacher.command(
                    env.world, env.receiver, Intent(Skill.RECEIVE, (r.x, r.y)), limits
                )
                label = np.array(
                    [
                        cmd.vx / limits.max_speed,
                        cmd.vy / limits.max_speed,
                        cmd.omega / limits.max_omega,
                    ]
                )
                samples.append(x)
                actions.append(label)
                if rng.random() < 0.35:
                    action[4:] = label
            _, _, done, truncated, info = env.step(action)
            if done or truncated:
                break
        # Give a stalled episode bounded weight in the fit.
        count = min(100, len(samples))
        for j in rng.choice(len(samples), count, replace=False):
            features.append(samples[j])
            labels.append(actions[j])
        records.append({"seed": seed + index, "reason": info["reason"], "samples": count})
    if not features:
        raise ValueError("No local reception samples were observed")
    x, y = np.array(features), np.array(labels)
    penalty = 1e-4
    weights = np.linalg.solve(x.T @ x + penalty * np.eye(x.shape[1]), x.T @ y)
    output.mkdir(parents=True, exist_ok=True)
    base_manifest = checkpoint.with_suffix(".json").read_bytes()
    base = json.loads(base_manifest)
    manifest = {
        "contract": RECEIVER_CONTRACT,
        "skill": base["skill"],
        "task": env.task_version,
        "config": asdict(env.cfg),
        "backend": env.backend.name,
        "base_checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
        "base_manifest_sha256": hashlib.sha256(base_manifest).hexdigest(),
        "pass_speed": base["pass_speed"],
        "receiver_weights": weights.tolist(),
        "receiver_range_m": RECEIVER_RANGE,
        "receiver_training_seeds": [seed, seed + episodes - 1],
        "method": "Frozen v2 transfer actor with a Webots imitation-fitted local receiver",
        "validated_for_hardware": False,
    }
    shutil.copyfile(checkpoint, output / "policy.zip")
    (output / "policy.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    report = dict(
        seed=seed,
        episodes=records,
        labeled_samples=len(x),
        ridge_penalty=penalty,
        teacher_probability=0.35,
        normalized_action_rmse=np.sqrt(np.mean((x @ weights - y) ** 2, axis=0)).tolist(),
    )
    (output / "receiver-fit.json").write_text(json.dumps(report, indent=2) + "\n")
