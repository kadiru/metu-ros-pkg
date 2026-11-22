"""Release overrides must affect the authorized carrier and the current pass only."""

import pytest
from roboturk.counterpass import DistributionGame
from roboturk.model import Ball, Command, Robot, RobotKey, World
from roboturk.pass_release import ReleaseGame, choose_release


def setup_game(monkeypatch, action):
    players = tuple(
        Robot(RobotKey(color, i), x, y)
        for color, positions in [
            ("blue", [(0, 0), (1, 0), (1, 1)]),
            ("yellow", [(0.3, 0.7), (2, 0), (2, 1)]),
        ]
        for i, (x, y) in enumerate(positions)
    )
    world = World(0, players, Ball(0.115, 0))
    game = ReleaseGame.__new__(ReleaseGame)
    game.__dict__.update(
        world=world,
        counter=dict(actor=0, team="blue", distribution=2, recipient=1, target=[1, 0], sample=39),
        release_plans={},
        tick=21,
        release_tick=None,
        flight=None,
        teams={"blue": {"pass_release": {"action": action}}},
        release_decisions=[],
        choice=object(),
    )

    def ready(self, out):
        self.release_plans[self.tick] = {"runner": self.counter["recipient"]}
        out[players[0].key] = Command(dribbler=1, kick=2.2)
        return out

    monkeypatch.setattr(DistributionGame, "counter_commands", ready)
    return game, players[0].key


@pytest.mark.parametrize("action", [0, 1, 2, 3])
def test_review_of_one_ready_kick(monkeypatch, action):
    game, key = setup_game(monkeypatch, action)
    cmd = ReleaseGame.counter_commands(game, {})[key]
    assert len(game.release_decisions) == 1
    assert (cmd.kick == 2.2) == (action == 0)
    assert (game.tick in game.release_plans) == (action == 0)
    assert game.counter["recipient"] == (2 if action == 2 else 1)
    assert (game.counter["distribution"] == 1) == (action == 3)
    if action == 1:
        for tick in range(22, 30):
            game.tick = tick
            command = ReleaseGame.counter_commands(game, {})[key]
            assert (command.kick == 2.2) == (tick == 29)
        assert len(game.release_decisions) == 1


def test_local_value_defaults_to_release_without_support():
    model = dict(
        examples=[[0, 0]],
        scale=[1, 1],
        advantages=[[0, 1, 0, 0]],
        neighbors=1,
        bandwidth=0.25,
        release_prior=0.1,
    )
    assert choose_release(model, [0, 0]) == 1
    assert choose_release(model, [4, 4]) == 0
