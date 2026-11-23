"""Fit observed-motion steering in paired native Webots self-play rollouts."""

import argparse
from dataclasses import asdict
import gzip
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from roboturk.open_motion import Steering
from open_motion_metrics import stats

ROOT = Path(__file__).resolve().parents[2]
PROTOCOL = json.loads((ROOT / "policies/open_motion_v1/protocol.json").read_text())


def play(path, seed, seconds, actor=None, switch=0, film=False):
    cmd = [
        sys.executable,
        str(ROOT / "scripts/run_five_roles.py"),
        str(path),
        "--seed",
        str(seed),
        "--seconds",
        str(seconds),
        "--motion-audit",
    ]
    if actor:
        cmd.extend(["--open-play-policy", str(actor), "--steer-from-tick", str(switch)])
    if film:
        cmd.append("--frame-images")
    subprocess.run(cmd, check=True)
    return read_game(path)


def read_game(path):
    with gzip.open(path / "roles.json.gz", "rt") as f:
        log = json.load(f)
    with np.load(path / "positions.npz") as f:
        return log, f["states"]


def save(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def incident(path, actor, number, film=False):
    root = PROTOCOL["incident_seed"]
    case = PROTOCOL["incidents"][number]
    switch = case["switch_tick"]
    seconds = case["end_tick"] // 25
    log, states = play(path, root, seconds, actor, switch if actor else 0, film)
    xy = states[:, 4:].reshape(-1, 10, 3)[:, :, :2]
    gaps = np.linalg.norm(xy[:, case["blue"]] - xy[:, 5 + case["yellow"]], axis=1)
    report = stats(log, states, switch)
    return dict(min_gap=float(min(gaps[switch:])), fast=report["fast_encounters"])


def train(dest):
    rng = np.random.default_rng(535)
    mean = np.array([0.37, 0.295, 0.63])
    spread = np.array([0.13, 0.045, 0.21])
    history = []
    for gen, seed in enumerate(PROTOCOL["training_seeds"]):
        group = []
        for i, params in enumerate(
            [
                mean,
                *np.clip(
                    rng.normal(mean, spread, (3, 3)), [0.16, 0.215, 0.32], [0.72, 0.415, 1.08]
                ),
            ]
        ):
            work = dest / f"gen{gen}-{i}"
            work.mkdir()
            weights = asdict(Steering(*params))
            actor = work / "actor.json"
            save(actor, weights)
            replay = [incident(work / f"incident{n}", actor, n) for n in range(2)]
            log, states = play(work / "game", seed, 24, actor)
            score = stats(log, states)
            reward = sum(10 * min(1, (r["min_gap"] - 0.18) / 0.15) - 5 * r["fast"] for r in replay)
            reward += (
                1.7 * len(score["passes"]) + 5 * len(score["goals"]) - 3 * score["fast_encounters"]
            )
            row = dict(
                generation=gen,
                index=i,
                seed=seed,
                weights=weights,
                incidents=replay,
                game=score,
                reward=reward,
            )
            group.append(row)
            history.append(row)
            save(dest / "history.json", history)
            print(
                gen,
                i,
                round(reward, 2),
                replay,
                len(score["passes"]),
                len(score["goals"]),
                flush=True,
            )
        best = sorted(group, key=lambda r: -r["reward"])[:2]
        mean = np.mean([list(r["weights"].values()) for r in best], axis=0)
        spread *= 0.65
    save(dest / "actor.json", best[0]["weights"])


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    args.output.mkdir(parents=True)
    train(args.output)
