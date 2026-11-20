"""Small synthetic optimizer integration checks, separate from native qualification."""

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("stable_baselines3")


@pytest.mark.parametrize(
    "decisions_only,categorical", [(False, False), (True, False), (True, True)]
)
def test_self_play_updates_only_learner_and_logs_training_resets(
    tmp_path, decisions_only, categorical
):
    from roboturk.backend import ReferenceBackend
    from roboturk.continuous_training import ShapeEnv, save, digest
    from roboturk.continuous import OBSERVATIONS
    from roboturk.continuous_selfplay import (
        RobotMatchEnv,
        freeze_motion,
        motion_hash,
        decision_head_hash,
    )
    from roboturk.self_play_training import SelfPlayPPO, actor_hash

    torch.set_num_threads(1)
    source = SelfPlayPPO(
        "MlpPolicy",
        ShapeEnv(),
        seed=611,
        device="cpu",
        n_steps=16,
        batch_size=8,
        n_epochs=1,
        policy_kwargs={"net_arch": [32, 32], "log_std_init": -5},
    )
    snapshot = save(source, tmp_path / "initial.zip", {"stage": "synthetic_test"})
    env = RobotMatchEnv(ReferenceBackend(), [snapshot], snapshot, 611, duration=0.04)
    from roboturk.continuous_distribution import ContinuousDecisionPolicy

    options = {"custom_objects": {"policy_class": ContinuousDecisionPolicy}} if categorical else {}
    model = SelfPlayPPO.load(snapshot["path"], env=env, device="cpu", learning_rate=1e-4, **options)
    if decisions_only:
        # Even restored momentum must not move frozen rows after masking.
        for p in model.policy.parameters():
            model.policy.optimizer.state[p] = {
                "step": torch.tensor(1.0),
                "exp_avg": torch.ones_like(p),
                "exp_avg_sq": torch.ones_like(p),
            }
        freeze_motion(model)
        assert not model.policy.optimizer.state
    motion_before, decision_before = motion_hash(model), decision_head_hash(model)
    x = np.random.default_rng(7).normal(0, 0.1, (64, OBSERVATIONS)).astype(np.float32)
    model.prepare_anchor(x, 611, [np.arange(len(x))])
    model.actor_mse_limit = 1
    model.actor_delta_limit = 2
    frozen = [actor_hash(env.teammate), actor_hash(env.opponents[0])]
    before = actor_hash(model)
    model.learn(total_timesteps=16)
    assert actor_hash(model) != before
    assert decision_head_hash(model) != decision_before
    if decisions_only:
        assert motion_hash(model) == motion_before
    if categorical:
        updated = save(model, tmp_path / "updated.zip", {"stage": "synthetic_test"})
        restored = SelfPlayPPO.load(updated["path"], device="cpu")
        assert isinstance(restored.policy, ContinuousDecisionPolicy)
        np.testing.assert_array_equal(
            model.predict(x, deterministic=True)[0], restored.predict(x, deterministic=True)[0]
        )
    assert frozen == [actor_hash(env.teammate), actor_hash(env.opponents[0])]
    assert digest(snapshot["path"]) == snapshot["sha256"]
    assert model.anchor_updates == 1 and model.anchor_rejections == 0
    assert len(env.records) == 8 and len(env.resets) == 9
    assert all(4300000 <= r["seed"] < 4400000 and r["opponent_index"] == 0 for r in env.resets)
    assert all(set(r["calls"].values()) == {2} and r["physics_checks"] == 2 for r in env.records)
    assert len({r["learner"] for r in env.resets}) == 4
    assert len({r["learner"] for r in env.resets[:4]}) == 4


def test_categorical_likelihood_ignores_frozen_motor_outputs():
    from roboturk.continuous_distribution import DecisionDistribution

    means = torch.zeros((128, 35), requires_grad=True)
    dist = DecisionDistribution(35).proba_distribution(means, torch.zeros(35))
    actions = dist.sample()
    np.testing.assert_array_equal(actions[:, 7:].detach(), means[:, 7:].detach())
    expected = torch.full((128,), -np.log(7), dtype=torch.float32)
    torch.testing.assert_close(dist.log_prob(actions), expected)
    altered = actions.detach().clone()
    altered[:, 7:] = 123
    torch.testing.assert_close(dist.log_prob(altered), dist.log_prob(actions))
    assert len(actions[:, :7].argmax(1).unique()) == 7
    (-dist.log_prob(actions).mean()).backward()
    assert torch.count_nonzero(means.grad[:, :7]) > 0
    assert torch.count_nonzero(means.grad[:, 7:]) == 0
    # Emitting a bounded categorical code prevents Box clipping from creating
    # ties when multiple learned raw scores exceed one.
    large = torch.zeros((1, 35))
    large[0, :2] = torch.tensor([2.0, 3.0])
    dist.proba_distribution(large, torch.zeros(35))
    assert dist.mode().clamp(-1, 1)[0, :7].argmax() == 1


def test_evaluation_records_actual_prediction_batches_per_team():
    from roboturk.backend import ReferenceBackend
    from roboturk.continuous import encode
    from roboturk.continuous_training import evaluate
    from roboturk.model import Command

    class Actor:
        def __init__(self):
            self.calls = 0

        def predict(self, observations, deterministic):
            assert len(observations) == 2 and deterministic
            self.calls += 1
            return np.stack([encode(0, Command())] * 2), None

    blue, yellow = Actor(), Actor()
    result = evaluate(ReferenceBackend(), blue, yellow, 2, 4400000, 0.04)
    assert blue.calls == yellow.calls == 4
    assert all(m["neural_inference_batches"] == {"blue": 2, "yellow": 2} for m in result["matches"])
