"""Native correction data and choice-conditioned intent-head preparation."""

from pathlib import Path
import json
import numpy as np
import torch
from .strategy_heads import expand
from .strategy_training import collect, load, write, digest, save, evaluate


def fit_heads(paths, parent, output, seed, epochs=250):
    torch.set_num_threads(1)
    torch.manual_seed(seed)
    data = [np.load(p) for p in paths]
    banks = [json.loads(Path(p).with_name("collection.json").read_text())["motors"] for p in paths]
    signatures = [{k: v["sha256"] for k, v in bank.items()} for bank in banks]
    assert all(s == signatures[0] for s in signatures), "Do not mix different motor-bank datasets"
    x = np.concatenate([d["observations"] for d in data])
    y = np.concatenate([d["actions"] for d in data])
    modes = y[:, :10].argmax(1)
    setting = (modes >= 7) & ((x[:, 122:132].argmax(1) != modes) | (y[:, 16] > 0))
    groups = [g for i in range(10) if len(g := np.flatnonzero(modes == i))]
    groups += [g for i in range(7, 10) if len(g := np.flatnonzero(setting & (modes == i)))]
    previous = load(parent)
    model = expand(parent, seed) if previous.policy.action_net.out_features == 17 else previous
    model.set_random_seed(seed)
    before = model.predict(x[:128], deterministic=True)[0]
    old = load(parent).predict(x[:128], deterministic=True)[0]
    assert np.array_equal(before[:, :10], old[:, :10])
    special = old[:, :10].argmax(1) >= 7
    np.testing.assert_allclose(before[special, 10:], old[special, 10:], atol=2e-6)
    inputs, labels, setting = torch.as_tensor(x), torch.as_tensor(y), torch.as_tensor(setting)
    opt = torch.optim.Adam(model.actor_parameters(), lr=2e-4)
    rng = np.random.default_rng(seed)
    losses = []
    for epoch in range(epochs):
        ids = np.concatenate([rng.choice(g, 2048, replace=True) for g in groups])
        rng.shuffle(ids)
        values = []
        for part in np.array_split(ids, max(1, len(ids) // 512)):
            pred = model.policy.get_distribution(inputs[part]).distribution.mean
            kinds = labels[part, :10].argmax(1)
            loss = torch.nn.functional.cross_entropy(pred[:, :10] * 3, kinds)
            targets = pred[:, 10:].reshape(-1, 3, 7)[
                torch.arange(len(part)), (kinds - 7).clamp(0, 2)
            ]
            special = kinds >= 7
            if special.any():
                loss = loss + 3 * (targets[special, 6] - labels[part][special, 16]).square().mean()
            chosen = setting[part]
            if chosen.any():
                loss = (
                    loss + 30 * (targets[chosen, :6] - labels[part][chosen, 10:16]).square().mean()
                )
            opt.zero_grad()
            loss.backward()
            opt.step()
            values.append(loss.item())
        losses.append(float(np.mean(values)))
        if (epoch + 1) % 25 == 0:
            print("Conditional intent fit", seed, epoch + 1, epochs, losses[-1], flush=True)
    model.policy.optimizer.state.clear()
    save(
        model,
        output,
        {
            "seed": seed,
            "method": "native DAgger with separate receive/redirect/clear intent heads",
            "raw_actor_outputs": 31,
            "deployed_action_values": 17,
            "epochs": epochs,
            "parent": str(parent),
            "parent_sha256": digest(parent),
            "datasets": [{"path": str(p), "sha256": digest(p)} for p in paths],
            "motors": banks[0],
            "target_loss": "selected skill head, only entry/positive refresh; refresh score on active skill",
            "loss_by_epoch": losses,
            "counts": {str(i): int((modes == i).sum()) for i in range(10)},
        },
    )
    return model


def adapt(backend, bank, output):
    out = Path(output)
    if out.exists():
        raise ValueError("Retain all adaptation attempts")
    out.mkdir(parents=True)
    root = out.parent
    for i, seed in enumerate((811, 822)):
        student = load(root / f"league{seed}-v1/generation2/policy.zip")
        collect(backend, bank, out / f"dagger{seed}", 32, 6121000 + 100 * i, 12, student, 0.8)
    paths = [
        root / "prepare-v1/teacher/data.npz",
        root / "prepare-v1/dagger811/data.npz",
        root / "prepare-v1/dagger822/data.npz",
        root / "refine-v2/teacher/data.npz",
        out / "dagger811/data.npz",
        out / "dagger822/data.npz",
    ]
    for seed in (811, 822):
        parent = root / f"refine-v2/seed{seed}/refit.zip"
        model = fit_heads(paths, parent, out / f"seed{seed}/initial.zip", seed)
        write(
            out / f"seed{seed}/development.json",
            evaluate(backend, bank, [model, model], 16, 6302000, 20),
        )
    write(
        out / "complete.json",
        {
            "seeds": [811, 822],
            "native_correction_matches_per_seed": 32,
            "mix": 0.8,
            "raw_actor_outputs": 31,
            "datasets": [{"path": str(p), "sha256": digest(p)} for p in paths],
        },
    )
