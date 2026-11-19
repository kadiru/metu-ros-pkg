"""Optional training tests; core environment tests run without PyTorch."""

from pathlib import Path
import numpy as np
import pytest

torch = pytest.importorskip("torch")
sb3 = pytest.importorskip("stable_baselines3")


@pytest.mark.parametrize(
    "role,relative",
    [
        ("defender", "learned_opponents_v1/defender/policy.zip"),
        ("keeper", "learned_opponents_v1/keeper/policy.zip"),
        ("transfer", "learned_opponents_play_v1/transfer/policy.zip"),
        ("finish", "learned_opponents_play_v1/finish/policy.zip"),
    ],
)
def test_expansion_preserves_source_actor_with_new_inputs_zero(role, relative):
    from roboturk.self_play_training import new_model

    source = Path(__file__).resolve().parents[1] / "policies" / relative
    original = sb3.PPO.load(source, device="cpu")
    expanded = new_model(role, source, 217)
    rng = np.random.default_rng(17)
    x = rng.normal(0, 0.2, (8, original.observation_space.shape[0])).astype(np.float32)
    extra = np.zeros((8, expanded.observation_space.shape[0]), np.float32)
    extra[:, : x.shape[1]] = x
    with torch.no_grad():
        expected = original.policy.get_distribution(torch.as_tensor(x)).distribution.mean
        actual = expanded.policy.get_distribution(torch.as_tensor(extra)).distribution.mean
    torch.testing.assert_close(actual, expected, atol=1e-6, rtol=1e-5)
