"""Structural checks only; native self-play establishes performance separately."""

from dataclasses import replace
import math
import numpy as np
import pytest
from roboturk.model import Command, Referee
from roboturk.continuous import KEYS, encode
from roboturk.strategy_options import StrategyMatch, formation, decode_intent, intent_values


class Replay:
    name = "synthetic_replay"

    def reset(self, world):
        self.world = world
        self.commands = None
        return world

    def step(self, commands):
        self.commands = commands
        self.last_substep_worlds = [
            replace(self.world, time=self.world.time + 0.005 * (i + 1)) for i in range(4)
        ]
        self.world = self.last_substep_worlds[-1]
        self.last_kick_events = []
        return self.world


class Motor:
    def predict(self, x, deterministic=True):
        return np.array([0, 0, 0, -1], np.float32), None


class Bank:
    skills = {k: Motor() for k in ("receive", "redirect", "clear")}

    def base_actions(self, match):
        return np.stack([encode(0, Command())] * 4)


def test_all_formations_and_observation_shapes():
    for seed in range(6100000, 6100064):
        w, f = formation(seed)
        assert len(w.robots) == 4 and set(r.key for r in w.robots) == set(KEYS)
        assert (
            min(
                math.hypot(a.x - b.x, a.y - b.y)
                for i, a in enumerate(w.robots)
                for b in w.robots[i + 1 :]
            )
            > 0.205
        )
        m = StrategyMatch(Replay(), Bank())
        m.reset(seed)
        assert m.observations().shape == (4, 162)
        assert m.start_family == f


def test_neural_choice_controls_option_entry_retention_and_exit():
    m = StrategyMatch(Replay(), Bank())
    m.reset(6100001)
    a = np.zeros((4, 17), np.float32)
    a[:, 0] = 0.8
    a[0, 0] = 0
    a[0, 7] = 0.8
    a[0, 12] = 1
    m.step(a)
    original = m.options[KEYS[0]]
    a[0, 10] = 0.7
    m.step(a)
    assert (
        m.options[KEYS[0]] is original
        and original.intent.point[0] != m.world.robot(KEYS[0]).x + 0.7
    )
    assert m.motor_calls["receive"] == 2
    a[0, 7] = 0
    a[0, 0] = 0.8
    m.step(a)
    assert m.options[KEYS[0]] is None
    assert [e["type"] for e in m.option_events] == ["option_start", "option_end"]


def test_target_codec_and_referee_gate():
    w, _ = formation(6100000)
    v = np.array([0.1, -0.1, 0.6, 0.8, 0.4, -0.5], np.float32)
    for key in KEYS:
        intent = decode_intent(w, key, "redirect", v)
        np.testing.assert_allclose(intent_values(w, key, intent), v, atol=1e-7)
    backend = Replay()
    m = StrategyMatch(backend, Bank())
    m.reset(6100000)
    m.world = backend.world = replace(m.world, referee=Referee.HALT)
    a = np.zeros((4, 17), np.float32)
    a[:, 8] = 0.8
    a[:, 12] = 1
    m.step(a)
    assert all(c == Command() for c in backend.commands.values())
    with pytest.raises(ValueError):
        m.step(np.zeros((4, 15)))
