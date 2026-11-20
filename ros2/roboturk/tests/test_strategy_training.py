# ruff: noqa: E402 -- optional RL dependencies are absent in the minimal core job.
"""Strategic PPO likelihood and frozen target-function contract."""

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("stable_baselines3")
from roboturk.strategy_training import SelectorDistribution, SelectorPolicy, ShapeEnv
from roboturk.strategy_selfplay import freeze_targets, target_hash
from roboturk.self_play_training import SelfPlayPPO


def test_categorical_likelihood_explores_only_behavior():
    torch.manual_seed(19)
    means = torch.zeros((4000, 17))
    means[:, 2] = 0.8
    means[:, 10:] = torch.tensor([0.1, -0.2, 0.3, 0.4, 0.5, -0.6, 0.7])
    distribution = SelectorDistribution(17).proba_distribution(means, torch.zeros(17))
    sampled = distribution.sample()
    assert torch.equal(sampled[:, 10:], means[:, 10:])
    choices = sampled[:, :10].argmax(1)
    assert len(choices.unique()) == 10
    observed = (choices == 2).float().mean().item()
    expected = torch.softmax(means[0, :10] * 3, 0)[2].item()
    assert abs(observed - expected) < 0.035
    torch.testing.assert_close(
        distribution.log_prob(sampled),
        torch.log_softmax(means[:, :10] * 3, 1).gather(1, choices[:, None]).squeeze(1),
    )
    assert torch.equal(distribution.mode()[:, :10].argmax(1), torch.full((4000,), 2))


def test_optimizer_cannot_change_target_function():
    torch.set_num_threads(1)
    model = SelfPlayPPO(
        SelectorPolicy,
        ShapeEnv(),
        n_steps=8,
        batch_size=4,
        policy_kwargs={"net_arch": {"pi": [256, 256], "vf": [256, 256]}},
        seed=811,
    )
    x = torch.tensor(np.random.default_rng(15).normal(size=(16, 162)), dtype=torch.float32)
    # Populate optimizer momentum before freezing to catch stale-state leakage.
    for _ in range(2):
        loss = model.policy.get_distribution(x).distribution.mean.square().sum()
        model.policy.optimizer.zero_grad()
        loss.backward()
        model.policy.optimizer.step()
    freeze_targets(model)
    before = target_hash(model)
    choices = model.policy.action_net.weight[:10].detach().clone()
    for _ in range(3):
        means = model.policy.get_distribution(x).distribution.mean
        loss = (means - 1).square().mean()
        model.policy.optimizer.zero_grad()
        loss.backward()
        model.policy.optimizer.step()
    assert target_hash(model) == before
    assert not torch.equal(model.policy.action_net.weight[:10], choices)
