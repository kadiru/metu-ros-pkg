"""A receiver-value head follows the candidate features when teammate order changes."""

import numpy as np
from roboturk.distribution_value import decision, design


def test_shared_receiver_weights_follow_the_route():
    carrier = [0.6, 0.7, 0.4, 0.3, 0.2, 0.5]
    near = [0.2, 0.4, 0.3, 0.6, 1.2, 0.4, 0.2]
    open_lane = [0.8, 0.9, 0.4, 0.8, 1.5, 0.6, 0.3]
    x = carrier + near + open_lane
    direct = np.zeros_like(design(x, 0))
    receiver = np.zeros_like(design(x, 2))
    receiver[7] = 1
    model = {"heads": [direct, direct, receiver]}
    assert decision(model, x) == 3
    assert decision(model, carrier + open_lane + near) == 2


def test_equal_action_values_preserve_the_incumbent():
    x = [0.0] * 20
    model = {"heads": [np.zeros_like(design(x, a)) for a in (0, 1, 2)]}
    assert decision(model, x) == 0
