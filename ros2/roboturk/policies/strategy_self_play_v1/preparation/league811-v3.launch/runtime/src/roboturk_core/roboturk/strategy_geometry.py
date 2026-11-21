"""Learned strategic readouts with direct geometric feature access.

The observation and deployed-action contracts stay unchanged. All target and
choice coefficients are trained. Unit velocity and its goal-axis projection are
state features; this module supplies no tactical choice, target or handoff rule.
"""

import torch
from .strategy_heads import IntentHeadPolicy
from .strategy_training import ShapeEnv, load
from .self_play_training import SelfPlayPPO


def grounded_refresh_labels(x, labels):
    """Correct historical refresh labels from measured geometry, retaining raw data."""
    import numpy as np

    labels = labels.copy()
    # Coaching margin above the unchanged 0.5 s reception metric; deployment
    # still learns its own exit decision, with no timed handoff in the runtime.
    holding = (x[:, 129] > 0.5) & (x[:, 139] > 0) & (x[:, 139] < 0.8) & (x[:, 7] > 0.5)
    labels[holding, :10] = -0.8
    labels[holding, 7] = 0.8
    labels[holding, 10:16] = x[holding, 132:138]
    labels[holding, 16] = -0.8
    mode = labels[:, :10].argmax(1)
    incoming = -x[:, 38:40] / np.maximum(np.linalg.norm(x[:, 38:40], axis=1, keepdims=True), 1e-6)
    goal = x[:, 157:159]
    projected_goal = (goal * incoming).sum(axis=1, keepdims=True) * incoming
    for kind, heading, projection in ((7, incoming, incoming), (8, goal, projected_goal)):
        mask = (mode == kind) & (labels[:, 16] > 0)
        labels[mask, 10:12] = np.clip(x[mask, 151:153] + 0.115 * projection[mask], -1, 1)
        labels[mask, 12:14] = heading[mask]
    return labels


class GeometryEncoder(torch.nn.Module):
    def __init__(self, learned):
        super().__init__()
        self.learned = learned

    def forward(self, x):
        velocity = x[:, 38:40]
        incoming = -velocity / velocity.norm(dim=1, keepdim=True).clamp_min(1e-6)
        goal = x[:, 157:159]
        projected_goal = (goal * incoming).sum(dim=1, keepdim=True) * incoming
        return torch.cat([self.learned(x), x, incoming, projected_goal], dim=1)


class GeometryPolicy(IntentHeadPolicy):
    def _build(self, lr_schedule):
        super()._build(lr_schedule)
        self.mlp_extractor.policy_net = GeometryEncoder(self.mlp_extractor.policy_net)
        self.mlp_extractor.latent_dim_pi += 166
        self.action_net = torch.nn.Linear(self.mlp_extractor.latent_dim_pi, 31)
        self.optimizer = self.optimizer_class(
            self.parameters(), lr=lr_schedule(1), **self.optimizer_kwargs
        )


def expand_geometry(parent, seed):
    old = load(parent)
    assert old.policy.action_net.out_features == 31
    assert old.policy.action_net.in_features == 256
    model = SelfPlayPPO(
        GeometryPolicy,
        ShapeEnv(),
        seed=seed,
        device="cpu",
        n_steps=1024,
        batch_size=256,
        n_epochs=5,
        learning_rate=3e-5,
        gamma=0.999,
        clip_range=0.1,
        policy_kwargs={"net_arch": {"pi": [256, 256], "vf": [256, 256]}, "log_std_init": -5},
    )
    model.policy.mlp_extractor.policy_net.learned.load_state_dict(
        old.policy.mlp_extractor.policy_net.state_dict()
    )
    model.policy.mlp_extractor.value_net.load_state_dict(
        old.policy.mlp_extractor.value_net.state_dict()
    )
    model.policy.value_net.load_state_dict(old.policy.value_net.state_dict())
    with torch.no_grad():
        model.policy.action_net.weight.zero_()
        model.policy.action_net.weight[:, :256].copy_(old.policy.action_net.weight)
        model.policy.action_net.bias.copy_(old.policy.action_net.bias)
        model.policy.log_std.copy_(old.policy.log_std)
    return model


def calibrate_targets(model, inputs, labels, setting):
    """Regularized supervised target regression after joint fitting; choices unchanged."""
    import numpy as np

    reports = []
    with torch.no_grad():
        for head in range(3):
            ids = torch.nonzero(setting & (labels[:, :10].argmax(1) == head + 7)).flatten()
            if len(ids) == 0:
                continue
            features = (
                model.policy.mlp_extractor.forward_actor(inputs[ids]).numpy().astype(np.float64)
            )
            x = np.column_stack([features, np.ones(len(features))])
            y = labels[ids, 10:16].numpy().astype(np.float64)
            entry = (inputs[ids, 122:132].argmax(1) != head + 7).numpy()
            scale = np.ones(len(ids))
            if entry.any() and (~entry).any():
                scale[entry] = np.sqrt(len(ids) / (2 * entry.sum()))
                scale[~entry] = np.sqrt(len(ids) / (2 * (~entry).sum()))
            weighted_x, weighted_y = x * scale[:, None], y * scale[:, None]
            start = 10 + 7 * head
            prior = np.vstack(
                [
                    model.policy.action_net.weight[start : start + 6].numpy().T,
                    model.policy.action_net.bias[start : start + 6].numpy(),
                ]
            )
            # Prefer simple linear geometry over modifying nonlinear residuals.
            penalty = np.full(x.shape[1], 0.01)
            penalty[:256] = 1.0
            matrix = weighted_x.T @ weighted_x + np.diag(penalty)
            weights = np.linalg.solve(matrix, weighted_x.T @ weighted_y + penalty[:, None] * prior)
            model.policy.action_net.weight[start : start + 6].copy_(
                torch.as_tensor(weights[:-1].T, dtype=torch.float32)
            )
            model.policy.action_net.bias[start : start + 6].copy_(
                torch.as_tensor(weights[-1], dtype=torch.float32)
            )
            reports.append(
                {
                    "head": head + 7,
                    "rows": len(ids),
                    "entry_rows": int(entry.sum()),
                    "refresh_rows": int((~entry).sum()),
                    "target_mse_before": float(np.mean((x @ prior - y) ** 2)),
                    "target_mse_after": float(np.mean((x @ weights - y) ** 2)),
                }
            )
    return reports
