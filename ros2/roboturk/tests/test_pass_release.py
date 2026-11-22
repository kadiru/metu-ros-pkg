"""Release overrides must affect the authorized carrier and the current pass only."""

import json
from pathlib import Path

import pytest
from roboturk.model import Ball, Command, Robot, RobotKey, World


def setup_game(monkeypatch, action):
    pytest.importorskip("torch", reason="The native 3v3 game loads the learned pass motor")
    from roboturk.counterpass import DistributionGame
    from roboturk.pass_release import ReleaseGame

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
    cmd = game.counter_commands({})[key]
    assert len(game.release_decisions) == 1
    assert (cmd.kick == 2.2) == (action == 0)
    assert (game.tick in game.release_plans) == (action == 0)
    assert game.counter["recipient"] == (2 if action == 2 else 1)
    assert (game.counter["distribution"] == 1) == (action == 3)
    if action == 1:
        for tick in range(22, 30):
            game.tick = tick
            command = game.counter_commands({})[key]
            assert (command.kick == 2.2) == (tick == 29)
        assert len(game.release_decisions) == 1


def test_local_value_defaults_to_release_without_support():
    pytest.importorskip("torch", reason="The release policy shares the native game runtime")
    from roboturk.pass_release import choose_release

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


@pytest.mark.parametrize(("folder", "side"), [("control", "previous"), ("evaluation", "trained")])
def test_published_release_counts_match_native_logs(folder, side):
    root = Path(__file__).resolve().parents[1] / "policies/pass_release_v1"
    comparison = json.loads((root / "comparison.json").read_text())
    expected = comparison["pass_counts"][side]
    attempts = goals = interceptions = completed = 0
    starts = set()
    for path in sorted((root / folder).glob("game-*.json")):
        g = json.loads(path.read_text())
        if g["blue_model"] != g["yellow_model"]:
            continue
        starts.add(g["seed"])
        for chain in g["recoveries"]:
            if chain["kind"] != "takeaway":
                continue
            goals += int("goal" in chain)
            for p in g["passes"]:
                if p["team"] == chain["team"] and chain["sample"] <= 4 * p["tick"] <= chain["end"]:
                    attempts += 1
                    interceptions += int(p["result"] == "intercepted")
                    completed += int(p["result"] == "complete" and 4 * p["end"] <= chain["end"])
    protocol = json.loads((root / "protocol.json").read_text())
    assert starts == set(
        range(protocol["fresh_first"], protocol["fresh_first"] + protocol["fresh_starts"])
    )
    assert (attempts, goals, interceptions, completed) == tuple(
        expected[k] for k in ("attempts", "goals", "intercepted", "completed")
    )
