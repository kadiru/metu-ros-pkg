#!/usr/bin/env python3
"""Recompute behavior changes from saved weights and the hashed training data."""

import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import torch
from roboturk.continuous_training import digest, load
from roboturk.continuous_selfplay import motion_hash
from roboturk.self_play_training import write


def audit(league):
    ledgers = list(league.glob("seed*/league.json"))
    assert len(ledgers) == 1
    ledger = json.loads(ledgers[0].read_text())
    data_path = league / "data.npz"
    data_hash = digest(data_path)
    data = np.load(data_path)
    x, y = data["observations"], data["actions"]
    modes = y[:, :7].argmax(1)
    groups = [np.flatnonzero(modes == i) for i in range(7)] + [
        np.flatnonzero(y[np.arange(len(y)), 10 + 4 * modes] > 0),
        np.flatnonzero((np.abs(x[:, 0]) * 4.5 > 3.8) | (np.abs(x[:, 1]) * 3 > 2.3)),
        np.flatnonzero(np.min(x[:, [54, 58, 62]], axis=1) * 5 < 0.5),
        np.flatnonzero(np.abs(x[:, 121]) < 0.5),
    ]
    groups = [g for g in groups if len(g)]
    result = []
    for generation, metadata in enumerate(ledger["updates"], 1):
        assert metadata["data_sha256"] == data_hash
        assert metadata["anchor_group_sizes"] == [len(g) for g in groups]
        rng = np.random.default_rng(metadata["training_seed"])
        indices = np.concatenate([rng.choice(g, 4096, replace=True) for g in groups])
        assert len(indices) == metadata["mode_probe_rows"]
        before_entry, after_entry = ledger["snapshots"][generation - 1 : generation + 1]
        for entry in (before_entry, after_entry):
            assert digest(entry["path"]) == entry["sha256"]
        before, after = load(before_entry["path"]), load(after_entry["path"])
        assert (
            motion_hash(before)
            == motion_hash(after)
            == metadata["motion_before"]
            == metadata["motion_after"]
        )
        inputs = torch.as_tensor(x)
        probe = inputs[indices]
        with torch.no_grad():
            targets = torch.cat(
                [before.policy.get_distribution(b).distribution.mean for b in inputs.split(4096)]
            )
            current = after.policy.get_distribution(probe).distribution.mean
            previous = before.policy.get_distribution(probe).distribution.mean
            raw_changes = int((targets[indices, :7].argmax(1) != current[:, :7].argmax(1)).sum())
            motor_delta = float((current[:, 7:] - previous[:, 7:]).abs().max())
        actual_before = before.predict(x[indices], deterministic=True)[0][:, :7].argmax(1)
        actual_after = after.predict(x[indices], deterministic=True)[0][:, :7].argmax(1)
        actual_changes = int((actual_before != actual_after).sum())
        assert raw_changes == metadata["deterministic_mode_probe_changes"]
        assert motor_delta < 1e-6
        result.append(
            {
                "generation": generation,
                "raw_mode_changes": raw_changes,
                "actual_decoded_mode_changes": actual_changes,
                "probe_rows": len(indices),
                "max_motor_output_delta": motor_delta,
                "motion_sha256": motion_hash(after),
                "before_sha256": before_entry["sha256"],
                "after_sha256": after_entry["sha256"],
                "probe_indices_sha256": hashlib.sha256(indices.tobytes()).hexdigest(),
            }
        )
    assert sum(r["actual_decoded_mode_changes"] for r in result) > 0
    return {
        "training_seed": ledger["training_seed"],
        "data_sha256": data_hash,
        "ledger_sha256": digest(ledgers[0]),
        "generations": result,
        "passed": True,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--league", type=Path, action="append", required=True)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        p.error("Preserve previous audit; use a new output path")
    torch.set_num_threads(1)
    result = [audit(path) for path in args.league]
    write(args.output, {"passed": True, "leagues": result})
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
