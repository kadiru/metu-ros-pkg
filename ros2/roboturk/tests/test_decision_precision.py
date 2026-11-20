"""Training safeguards, using synthetic tensors rather than simulator outcomes."""

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("stable_baselines3")


def test_tactical_gradient_updates_preserve_coordinate_function_with_adam_momentum():
    from roboturk.decision_precision import preserve_coordinates, coordinate_hash
    from roboturk.decision_training import new_manager

    torch.set_num_threads(1)
    model = new_manager(457)
    x = torch.randn(64, 75)

    def update():
        y = model.policy.get_distribution(x).distribution.mean
        loss = ((y - 0.5) ** 2).mean()
        model.policy.optimizer.zero_grad()
        loss.backward()
        model.policy.optimizer.step()

    update()  # Populate Adam momentum before applying the safeguard.
    before = model.policy.get_distribution(x).distribution.mean.detach().clone()
    original = preserve_coordinates(model)
    for _ in range(3):
        update()
    after = model.policy.get_distribution(x).distribution.mean.detach()
    assert original == coordinate_hash(model)
    assert torch.equal(before[:, 5:9], after[:, 5:9])
    assert not torch.equal(before[:, :5], after[:, :5])


def test_boundary_coaching_is_symmetric_and_preserves_interior_commands():
    from roboturk.decision_precision import boundary_labels
    from roboturk.model import Config

    x = np.zeros((3, 45), np.float32)
    x[:, 15] = 1
    x[:, 33] = [4.35 / 4.5, -4.35 / 4.5, 0]
    a = np.array([[0, 0.1, 0.2], [0, 0.1, 0.2], [0.3, -0.2, 0.1]], np.float32)
    original = a.copy()
    y = boundary_labels(x, a, Config())
    assert y[0, 0] < 0 < y[1, 0]
    assert y[0, 0] == pytest.approx(-y[1, 0])
    np.testing.assert_array_equal(y[:, 1:], a[:, 1:])
    np.testing.assert_array_equal(y[2], a[2])
    np.testing.assert_array_equal(original, a)
