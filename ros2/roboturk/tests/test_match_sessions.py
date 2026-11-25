import importlib
import json
from pathlib import Path
from unittest.mock import Mock

import pytest

from roboturk.match_limits import GAME_LIMITS


@pytest.fixture
def sessions(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts"))
    return importlib.import_module("match_sessions")


def test_match_launch_uses_selected_timing_and_preserves_preset_defaults(
    sessions, monkeypatch, tmp_path
):
    preset = tmp_path / "preset.json"
    preset.write_text(json.dumps(dict(mode="league_match", half_seconds=30, seconds=540)))
    manager = sessions.MatchSessions(tmp_path / "matches", preset, Mock())
    process = Mock()
    process.wait.return_value = process.poll.return_value = 0
    monkeypatch.setattr(sessions.subprocess, "Popen", Mock(return_value=process))

    game = manager.start(dict(mode="game", team="blue", half_seconds=60, periods=2))
    game_spec = json.loads((game.directory / "spec.json").read_text())
    assert (game_spec["half_seconds"], game_spec["periods"]) == (60, 2)
    assert game.limit_path is None

    limits = dict(GAME_LIMITS, max_kick=10)
    strategy = dict(strategy="balanced", pass_preference=0)
    practice = manager.start(
        dict(
            mode="practice",
            strategies=dict.fromkeys(("blue", "yellow"), strategy),
            limits=limits,
            half_seconds=300,
            periods=1,
        )
    )
    practice_spec = json.loads((practice.directory / "spec.json").read_text())
    assert (practice_spec["half_seconds"], practice_spec["periods"]) == (300, 1)
    assert practice_spec["seconds"] > 600
    assert json.loads(practice.limit_path.read_text()) == limits

    default = manager.start(dict(mode="game", team="yellow"))
    assert (default.options["half_seconds"], default.options["periods"]) == (30, 2)


@pytest.mark.parametrize(
    "invalid",
    [
        dict(half_seconds=0),
        dict(half_seconds=60.5),
        dict(half_seconds=True),
        dict(half_seconds=301),
        dict(periods=3),
        dict(periods="2"),
        dict(limits=GAME_LIMITS),
    ],
)
def test_game_rejects_invalid_timing_and_motion_overrides(sessions, invalid):
    with pytest.raises(ValueError):
        sessions.session_options(dict(mode="game", team="blue") | invalid)
