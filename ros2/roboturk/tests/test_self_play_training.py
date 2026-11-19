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


def test_training_spacing_labels_reduce_closing_and_preserve_far_motion():
    from roboturk.self_play_repair import spacing_labels

    x = np.zeros((2, 45), np.float32)
    x[:, 7], x[:, 15] = 1, 0.8
    x[:, 35], x[:, 39], x[:, 43] = 1, 1, 1
    x[0, 35] = 0.25 / 3
    teacher = np.array([[1, 0, 0.4], [1, 0, 0.4]], np.float32)
    reference = np.array([[0.9, 0.1, 0.2], [0.9, 0.1, 0.2]], np.float32)
    labels, distance = spacing_labels(x, teacher, reference)
    assert distance[0] == pytest.approx(0.25)
    assert labels[0, 0] <= 0 and labels[0, 2] == reference[0, 2]
    np.testing.assert_array_equal(labels[1], reference[1])
    x[0, 37] = -0.2  # Neighbor approaches at0.4m/s: learn to retreat.
    labels, _ = spacing_labels(x, teacher, reference)
    assert labels[0, 0] <= -0.5 + 1e-6
