"""Training safeguards, using synthetic tensors rather than simulator outcomes."""

import numpy as np
import pytest

torch = pytest.importorskip("torch")
pytest.importorskip("stable_baselines3")


def test_tactical_gradient_updates_preserve_geometry_regressor_with_adam_momentum():
    from roboturk.decision_precision import preserve_geometry_regressor, geometry_regressor_hash
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
    original = preserve_geometry_regressor(model)
    for _ in range(3):
        update()
    after = model.policy.get_distribution(x).distribution.mean.detach()
    assert original == geometry_regressor_hash(model)
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


def test_spacing_coaching_uses_relative_velocity_and_robot_frame():
    from roboturk.decision_precision import spacing_labels
    from roboturk.model import Config

    x = np.zeros((2, 45), np.float32)
    x[:, 15] = 1
    x[0, 7] = 1  # yaw0
    x[1, 6] = 1  # yaw pi/2; identical canonical geometry below
    x[0, 35:37] = [0.23 / 3, 0]
    x[1, 35:37] = [0, -0.23 / 3]
    x[0, 39:41] = [-2 / 3, 0]
    x[1, 39:41] = [0, 2 / 3]
    actions = np.tile([1.0, 0, 0.2], (2, 1)).astype(np.float32)
    result = spacing_labels(x, actions, Config())
    np.testing.assert_allclose(result[0], result[1], atol=1e-6)
    assert 0 <= result[0, 0] <= 0.061
    assert result[0, 2] == actions[0, 2]
    # A teammate moving away allows a greater safe closing velocity.
    x[0, 37] = 0.5 / Config().max_speed
    moving = spacing_labels(x[:1], actions[:1], Config())
    assert moving[0, 0] > result[0, 0]
