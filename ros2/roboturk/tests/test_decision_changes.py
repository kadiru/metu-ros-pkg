import importlib
import json
from pathlib import Path

import pytest


@pytest.fixture
def change_tool(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "scripts"))
    return importlib.import_module("decision_changes")


def request_value():
    return {
        "format": "roboturk.decision_change_request.v1",
        "title": "Crowded Yellow outlet",
        "team": "yellow",
        "intent": "Prefer an outlet a teammate can receive cleanly.",
        "acceptance": "Compare receptions on matched starts.",
        "evidence": {
            "format": "roboturk.telemetry_clip.v1",
            "source": {"controller_versions": {"teams": {"yellow": {"id": "team-wide@1.0.0"}}}},
            "range": {"start_tick": 89, "end_tick": 91},
            "review": {"tick": 90},
            "samples": [{"tick": 89}, {"tick": 90}, {"tick": 91}],
        },
    }


@pytest.mark.parametrize("sparse", [False, True])
def test_branch_copies_source_and_keeps_request_and_baseline_separate(change_tool, tmp_path, sparse):
    repo = tmp_path / "baseline"
    stack = repo / "ros2/roboturk"
    stack.mkdir(parents=True)
    git = change_tool.git
    git(repo, "init", "--initial-branch=master")
    code = stack / "controller.py"
    code.write_text("DECISION = 'baseline'\n")
    (repo / "historical.txt").write_text("Other project files\n")
    git(repo, "add", ".")
    git(repo, "-c", "user.name=Kadir Uyanik", "-c", "user.email=717445+kadiru@users.noreply.github.com", "commit", "-m", "Fixture baseline")
    commit = git(repo, "rev-parse", "HEAD")
    if sparse:
        git(repo, "sparse-checkout", "set", "--no-cone", "--stdin", data="/ros2/roboturk/\n")
    code.write_text("DECISION = 'uncommitted baseline edit'\n")

    request = request_value()
    created = change_tool.create_candidate(request, stack=stack)
    candidate = Path(created["worktree"])
    assert created["created"]
    assert (candidate / "ros2/roboturk/controller.py").read_text() == "DECISION = 'baseline'\n"
    assert (candidate / "historical.txt").exists() is not sparse
    assert git(repo, "branch", "--show-current") == "master"
    assert code.read_text() == "DECISION = 'uncommitted baseline edit'\n"
    record = Path(created["record"])
    assert json.loads(record.with_name("request.json").read_text()) == request
    manifest = json.loads(record.read_text())
    assert manifest["base_commit"] == commit
    assert manifest["recorded_release"] == "team-wide@1.0.0"
    assert manifest["candidate_version"] is None
    assert not change_tool.create_candidate(request, stack=stack)["created"]


def test_rejects_missing_decision_frame(change_tool):
    value = request_value()
    value["evidence"]["review"]["tick"] = 93
    with pytest.raises(ValueError, match="contain the decision frame"):
        change_tool.validate_request(value)


@pytest.mark.parametrize(
    ("layers", "breaks", "version"),
    [
        (["behavior"], False, "1.2.4"),
        (["behavior", "interaction"], False, "1.3.0"),
        (["interaction", "strategy"], False, "2.0.0"),
        (["behavior"], True, "2.0.0"),
    ],
)
def test_version_uses_highest_changed_layer(change_tool, layers, breaks, version):
    assert change_tool.next_version("1.2.3", layers, breaks_contract=breaks) == version
