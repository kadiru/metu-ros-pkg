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


def test_choice_uses_its_own_intent_and_remains_a_17_value_action():
    from roboturk.strategy_heads import IntentHeadDistribution, IntentHeadPolicy

    means = torch.zeros((3, 31))
    for head in range(3):
        means[:, 10 + 7 * head : 17 + 7 * head] = 0.1 * (head + 1)
    dist = IntentHeadDistribution(17).proba_distribution(means, torch.zeros(17))
    chosen = dist.chosen(torch.tensor([7, 8, 9]))
    assert chosen.shape == (3, 17)
    torch.testing.assert_close(chosen[:, 10:], torch.tensor([[0.1] * 7, [0.2] * 7, [0.3] * 7]))
    model = SelfPlayPPO(
        IntentHeadPolicy,
        ShapeEnv(),
        n_steps=8,
        batch_size=4,
        policy_kwargs={"net_arch": {"pi": [256, 256], "vf": [256, 256]}},
        seed=822,
    )
    assert model.predict(np.zeros((4, 162), np.float32), deterministic=True)[0].shape == (4, 17)
    freeze_targets(model)
    before = target_hash(model)
    x = torch.ones((8, 162))
    for _ in range(2):
        loss = model.policy.get_distribution(x).distribution.mean.square().sum()
        model.policy.optimizer.zero_grad()
        loss.backward()
        model.policy.optimizer.step()
    assert target_hash(model) == before


def test_conditional_head_checkpoint_round_trip(tmp_path):
    from roboturk.strategy_heads import IntentHeadPolicy
    from roboturk.strategy_training import save, load
    from roboturk.self_play_training import actor_hash

    model = SelfPlayPPO(
        IntentHeadPolicy,
        ShapeEnv(),
        n_steps=8,
        batch_size=4,
        policy_kwargs={"net_arch": {"pi": [256, 256], "vf": [256, 256]}},
        seed=811,
    )
    source = tmp_path / "policy.zip"
    save(model, source, {"raw_actor_outputs": 31})
    loaded = load(source)
    assert loaded.policy.action_net.out_features == 31
    assert actor_hash(loaded) == actor_hash(model)
    x = np.random.default_rng(41).normal(size=(32, 162)).astype(np.float32)
    np.testing.assert_array_equal(
        loaded.predict(x, deterministic=True)[0], model.predict(x, deterministic=True)[0]
    )


def test_geometry_encoder_preserves_checkpoint_and_frozen_target_contract(tmp_path):
    from roboturk.strategy_geometry import GeometryPolicy
    from roboturk.strategy_training import save, load

    model = SelfPlayPPO(
        GeometryPolicy,
        ShapeEnv(),
        n_steps=8,
        batch_size=4,
        seed=811,
        policy_kwargs={"net_arch": {"pi": [256, 256], "vf": [256, 256]}},
    )
    x = np.random.default_rng(83).normal(size=(16, 162)).astype(np.float32)
    x[0, 38:40] = 0
    before = model.predict(x, deterministic=True)[0]
    assert before.shape == (16, 17) and np.isfinite(before).all()
    path = tmp_path / "policy.zip"
    save(model, path, {"geometry_features": True})
    restored = load(path)
    np.testing.assert_array_equal(before, restored.predict(x, deterministic=True)[0])
    freeze_targets(restored)
    target = target_hash(restored)
    old_choices = restored.policy.action_net.weight[:10].detach().clone()
    for _ in range(2):
        loss = restored.policy.get_distribution(torch.as_tensor(x)).distribution.mean.square().sum()
        restored.policy.optimizer.zero_grad()
        loss.backward()
        restored.policy.optimizer.step()
    assert target_hash(restored) == target
    assert not torch.equal(old_choices, restored.policy.action_net.weight[:10])


def test_corrective_refresh_labels_do_not_copy_the_student_target():
    from roboturk.strategy_geometry import grounded_refresh_labels

    x = np.zeros((2, 162), np.float32)
    x[:, 38:40] = [0.4, 0.2]
    x[:, 151:153] = [0.03, -0.06]
    x[0, 132:138] = 0.9
    x[1, 132:138] = -0.7
    y = np.zeros((2, 17), np.float32)
    y[:, 7] = 0.8
    y[:, 16] = 0.8
    y[0, 10:16] = 0.7
    y[1, 10:16] = -0.8
    corrected = grounded_refresh_labels(x, y)
    np.testing.assert_array_equal(corrected[0, 10:14], corrected[1, 10:14])
    assert not np.array_equal(y[0, 10:14], y[1, 10:14])
    y[:, 16] = -0.8
    np.testing.assert_array_equal(grounded_refresh_labels(x, y), y)
