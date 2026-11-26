"""Choice-conditioned learned intents; the deployed action remains 17 values.

The actor has 10 choice logits plus three independent seven-value intent heads.
PPO can change a choice without accidentally retaining another skill's intent.
There are no tactical rules in this generic categorical parameter routing.
"""

import torch
from .strategy_training import SelectorPolicy, SelectorDistribution, ShapeEnv, load
from .self_play_training import SelfPlayPPO

RAW_OUTPUTS = 31


class IntentHeadDistribution(SelectorDistribution):
    def proba_distribution(self, mean_actions, log_std):
        if mean_actions.shape[1] != RAW_OUTPUTS:
            raise ValueError("Expected ten logits and three seven-value intent heads")
        # Normal is only a raw-mean container for the existing actor trust region.
        # Likelihood, entropy and sampling are exclusively categorical.
        self.distribution = torch.distributions.Normal(mean_actions, torch.ones_like(mean_actions))
        self.choices = torch.distributions.Categorical(logits=mean_actions[:, :10] * 3)
        return self

    def chosen(self, indices):
        means = self.distribution.mean
        actions = torch.zeros((len(means), 17), dtype=means.dtype, device=means.device)
        actions[:, :10] = -0.8
        rows = torch.arange(len(means), device=means.device)
        actions[rows, indices] = 0.8
        values = means[:, 10:].reshape(-1, 3, 7)[rows, (indices - 7).clamp(0, 2)]
        actions[:, 10:] = torch.where((indices >= 7)[:, None], values, torch.zeros_like(values))
        return actions


class IntentHeadPolicy(SelectorPolicy):
    def _build(self, lr_schedule):
        super()._build(lr_schedule)
        self.action_net = torch.nn.Linear(self.mlp_extractor.latent_dim_pi, RAW_OUTPUTS)
        torch.nn.init.orthogonal_(self.action_net.weight, gain=0.01)
        torch.nn.init.zeros_(self.action_net.bias)
        self.optimizer = self.optimizer_class(
            self.parameters(), lr=lr_schedule(1), **self.optimizer_kwargs
        )

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.action_dist = IntentHeadDistribution(17)


def expand(parent, seed):
    """Exact warm start: copy each old target function into all three new heads."""
    old = load(parent)
    assert old.policy.action_net.out_features == 17
    model = SelfPlayPPO(
        IntentHeadPolicy,
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
    model.policy.mlp_extractor.load_state_dict(old.policy.mlp_extractor.state_dict())
    model.policy.value_net.load_state_dict(old.policy.value_net.state_dict())
    with torch.no_grad():
        for parameter in ("weight", "bias"):
            source = getattr(old.policy.action_net, parameter)
            target = getattr(model.policy.action_net, parameter)
            target[:10].copy_(source[:10])
            for head in range(3):
                target[10 + 7 * head : 17 + 7 * head].copy_(source[10:17])
        model.policy.log_std.copy_(old.policy.log_std)
    return model
