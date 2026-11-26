"""Counterfactual skill labels retain the original selected behavior and motion."""

from pathlib import Path
from types import SimpleNamespace
import numpy as np
from roboturk.continuous import KEYS, Memory, initial_world
from roboturk.continuous_teacher import ContinuousTeacher
from roboturk.model import Config


def test_all_head_labels_preserve_selected_demonstration(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).parents[1] / "scripts"))
    from label_allheads_continuous import all_heads

    teacher = ContinuousTeacher()
    for seed in range(12):
        world = initial_world(
            np.random.default_rng(seed),
            -1 if seed % 2 else 1,
            possession_team=(None, "blue", "yellow")[seed % 3],
        )
        memory = Memory()
        memory.reset(world)
        match = SimpleNamespace(world=world, memory=memory, cfg=Config())
        for key in KEYS:
            original, expanded = teacher.action(match, key), all_heads(match, key)
            mode = original[:7].argmax()
            np.testing.assert_array_equal(original[:7], expanded[:7])
            np.testing.assert_array_equal(
                original[7 + 4 * mode : 11 + 4 * mode], expanded[7 + 4 * mode : 11 + 4 * mode]
            )
            assert expanded.shape == (35,) and np.isfinite(expanded).all()
            assert np.all(np.abs(expanded) <= 1)
            # Every alternative head receives an explicit release/no-release label.
            assert np.all(np.isclose(np.abs(expanded[10::4]), 0.8))
