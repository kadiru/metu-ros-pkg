#!/usr/bin/env python3
"""Train a pass motor from recorded native states and explicit coaching labels."""

import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import torch
from replay_strategy import world
from audit_strategy import read
from roboturk.continuous import KEYS, Memory, encode
from roboturk.model import Config
from roboturk.strategy_pass import observation, pass_command, network


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--input", nargs="+", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--epochs", type=int, default=200)
    a = p.parse_args()
    assert not a.output.exists()
    a.output.mkdir(parents=True)
    for path in a.input:
        assert all(6100000 <= m["seed"] < 6200000 for m in read(path)["matches"]), (
            "Motor preparation must use training-range native states only"
        )
    xs, ys = [], []
    cfg = Config()
    for path in a.input:
        data = read(path)
        for m in data["matches"]:
            memory = Memory()
            state = world(m["initial_snapshot"], m["attack_sign"])
            memory.reset(state)
            resets = {
                e["step"]: e["snapshot"]
                for e in m["option_events"]
                if e["type"] == "restart_snapshot"
            }
            for step in range(len(m["decision_trace"])):
                if step in resets:
                    state = world(resets[step], m["attack_sign"])
                    memory.reset(state)
                match = SimpleNamespace(world=state, memory=memory, cfg=cfg)
                if step % 2 == 0:
                    for key in KEYS:
                        r = state.robot(key)
                        if np.hypot(r.x - state.ball.x, r.y - state.ball.y) < 0.5:
                            xs.append(observation(match, key))
                            ys.append(encode(2, pass_command(match, key))[15:19])
                samples = [
                    world(row, m["attack_sign"])
                    for row in m["physics_trace"][4 * step : 4 * step + 4]
                ]
                memory.observe(samples, [], cfg)
                state = samples[-1]
        print("Native pass labels", path, len(xs), flush=True)
    x, y = np.asarray(xs, np.float32), np.asarray(ys, np.float32)
    np.savez_compressed(a.output / "data.npz", observations=x, actions=y)
    torch.set_num_threads(1)
    torch.manual_seed(833)
    model = network()
    optimizer = torch.optim.Adam(model.parameters(), lr=3e-4)
    inputs, labels = torch.as_tensor(x), torch.as_tensor(y)
    fire = y[:, 3] > 0
    groups = [
        np.flatnonzero(fire),
        np.flatnonzero(~fire),
        np.flatnonzero((~fire) & (x[:, 10] > 0.5) & (abs(x[:, 7]) < 0.7)),
    ]
    groups = [g for g in groups if len(g)]
    rng = np.random.default_rng(833)
    losses = []
    for epoch in range(a.epochs):
        ids = np.concatenate([rng.choice(g, 4096, replace=True) for g in groups])
        rng.shuffle(ids)
        values = []
        for part in np.array_split(ids, len(ids) // 512):
            pred = model(inputs[part])
            target = labels[part]
            loss = 20 * (
                pred[:, :3] - target[:, :3]
            ).square().mean() + torch.nn.functional.binary_cross_entropy_with_logits(
                pred[:, 3] * 4, (target[:, 3] > 0).float()
            )
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            values.append(loss.item())
        losses.append(float(np.mean(values)))
        if (epoch + 1) % 25 == 0:
            print("Pass motor fit", epoch + 1, a.epochs, losses[-1], flush=True)
    checkpoint = a.output / "policy.pt"
    torch.save(model.state_dict(), checkpoint)

    def sha(p):
        return hashlib.sha256(p.read_bytes()).hexdigest()

    metadata = {
        "contract": "roboturk.strategy_pass.v1",
        "training_seed": 833,
        "epochs": a.epochs,
        "rows": len(x),
        "fire_rows": int(fire.sum()),
        "method": "Native recorded-state supervision; no standalone reliability claim",
        "checkpoint_sha256": sha(checkpoint),
        "dataset_sha256": sha(a.output / "data.npz"),
        "sources": [{"path": str(p), "sha256": sha(p)} for p in a.input],
        "source_scripts": {
            str(p): sha(p)
            for p in [
                Path(__file__),
                Path(__file__).resolve().parents[1] / "src/roboturk_core/roboturk/strategy_pass.py",
            ]
        },
        "loss_by_epoch": losses,
    }
    checkpoint.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")


if __name__ == "__main__":
    main()
