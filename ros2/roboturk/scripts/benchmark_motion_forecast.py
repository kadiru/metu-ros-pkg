"""Compare causal one-second XY forecasts on full saved 5v5 matches."""

import argparse
from hashlib import sha256
import json
from pathlib import Path

import numpy as np

from roboturk.motion_forecast import (
    ACCELERATION_NOISE,
    HORIZON_S,
    MEASUREMENT_STD_M,
    WARMUP_S,
    open_play_segments,
    position_errors,
    replay_forecasts,
    ticks_for,
)
from role_viewer import archive_folder, read_recording


CHECKPOINTS = np.array([0.10, 0.50, HORIZON_S])
MODELS = ("current", "kalman")


def summarize(checkpoints, path):
    endpoint = checkpoints[:, -1, :]
    return {
        name: {
            "mean_error_m": dict(
                zip(("100 ms", "500 ms", "1 s"), checkpoints[..., model].mean(axis=0).tolist())
            ),
            "ade_m": float(path[..., model].mean()),
            "p95_1s_m": float(np.percentile(endpoint[:, model], 95)),
        }
        for model, name in enumerate(MODELS)
    } | {"cases": int(len(endpoint))}


def evaluate(folder, noise_mm, noise_seed):
    archive = archive_folder(folder)
    log, samples = read_recording(archive)
    frames, keys, dt = log["frames"], log["keys"], log["dt"]
    if len(keys) != 10 or frames[-1].get("stage") != "POST_GAME":
        raise ValueError(f"Expected a complete officiated 5v5 match in {archive}")
    truth = samples[:, 4:].reshape(len(samples), len(keys), 3)[..., :2]
    segments = open_play_segments(frames, dt)
    path_times = np.minimum(HORIZON_S, dt * np.arange(1, ticks_for(HORIZON_S, dt) + 1))
    provenance = {
        "seed": log["seed"],
        "dt": dt,
        "frames": len(frames),
        "score": log["score"],
        "input_sha256": {
            name: sha256((archive / name).read_bytes()).hexdigest()
            for name in ("positions.npz", "roles.json.gz")
        },
    }
    results = {}
    for condition, measurement in (
        ("recorded", truth),
        (
            "noisy",
            truth
            + np.random.default_rng([noise_seed, log["seed"]]).normal(
                0.0, noise_mm / 1000, truth.shape
            ),
        ),
    ):
        forecasts = replay_forecasts(measurement, dt, segments)
        if not forecasts.valid.any():
            raise ValueError(f"No full one-second open-play windows in {archive}")

        def errors_at(times):
            # Collect one row per forecast origin and robot.
            return position_errors(forecasts, truth, times, dt).transpose(0, 2, 1, 3).reshape(
                -1, len(times), len(MODELS)
            )

        checkpoints, path = errors_at(CHECKPOINTS), errors_at(path_times)
        results[condition] = (
            summarize(checkpoints, path) | {"origins": int(forecasts.valid.sum())},
            checkpoints,
            path,
        )
    return provenance, results


def match_argument(value):
    name, separator, folder = value.partition("=")
    if not separator or not name.strip() or not folder:
        raise argparse.ArgumentTypeError("Use NAME=RECORDING for each match")
    return name, Path(folder)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("matches", nargs="+", type=match_argument)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--noise-mm", default=10.0, type=float)
    parser.add_argument("--noise-seed", default=61025, type=int)
    args = parser.parse_args()
    if not np.isfinite(args.noise_mm) or args.noise_mm <= 0 or args.noise_seed < 0:
        parser.error("Use positive finite noise and a non-negative seed")
    if len(dict(args.matches)) != len(args.matches):
        parser.error("Give each recording a distinct name")
    conditions = ("recorded", "noisy")
    totals = {condition: [] for condition in conditions}
    inputs = []
    for name, folder in args.matches:
        provenance, results = evaluate(folder, args.noise_mm, args.noise_seed)
        inputs.append(
            {"match": name, **provenance}
            | {condition: results[condition][0] for condition in conditions}
        )
        for condition in conditions:
            totals[condition].append(results[condition][1:])
    for condition, errors in totals.items():
        totals[condition] = summarize(*(np.concatenate(group) for group in zip(*errors, strict=True)))
    result = {
        "horizon_s": HORIZON_S,
        "warmup_s": WARMUP_S,
        "kalman": {
            "measurement_std_m": MEASUREMENT_STD_M,
            "acceleration_noise_m2_s3": ACCELERATION_NOISE,
            "initial_velocity_std_mps": 1.0,
        },
        "input_noise_std_mm": args.noise_mm,
        "input_noise_seed": args.noise_seed,
        "matches": inputs,
        "pooled": totals,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps(totals, indent=2))


if __name__ == "__main__":
    main()
