from types import SimpleNamespace
from unittest.mock import patch

import pytest

pytest.importorskip("torch")

from roboturk.early_outlet import EarlyOutletGame
from roboturk.finish_touch import CleanRecoveryGame
from roboturk.model import Ball, Command, Robot, RobotKey, World


@pytest.mark.parametrize("kick", [0, 4.0])
def test_post_shield_pivot_preserves_an_actual_shot(kick):
    actor = Robot(RobotKey("blue", 0), 0.0, 0.0)
    rival = Robot(RobotKey("yellow", 2), 0.55, -0.15)
    game = object.__new__(CleanRecoveryGame)
    game.world = World(0, (actor, rival), Ball(0.115, 0))
    game.delivery = game.finished_delivery = SimpleNamespace()
    game.runner = actor.key
    roller = dict(
        rescue_look=0.08,
        rescue_edge=0.8,
        retouch_gate=0.9,
        rescue_age=0,
        rescue_side=0.45,
        guard_age=0,
        rescue_drive=0.35,
        rescue_step=0.245,
        rescue_grip=21,
        rescue_turn=5.0,
        cap=1.16,
    )
    game.teams = {"blue": dict(finish_roller=roller)}
    cmd = Command(0.0, 1.15, 0.0, 1.0, kick)
    with patch.object(EarlyOutletGame, "commands", return_value={actor.key: cmd}):
        result = game.commands()[actor.key]
    if kick:
        assert result is cmd
    else:
        assert result.dribbler == 1 and result.kick == 0 and abs(result.vy) < 1.15
