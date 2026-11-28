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


def test_match_timeline_replaces_parent_future_and_reindexes_trial_events(change_tool):
    timeline = importlib.import_module("decision_timeline")
    parent = dict(
        keys=[["blue", 0]], dt=0.04, intro={"frame": 0, "robot": 0},
        frames=[{"t": 0.0}, {"t": 0.04}, {"t": 0.08}], positions=[[0], [1], [2]],
        events=[[0, "Shared", "period", None], [1, "Original pass", "pass", "blue"]],
    )
    trial = dict(
        keys=parent["keys"], dt=parent["dt"], frames=[{"t": 0.0}, {"t": 0.04}],
        positions=[[1], [3]], events=[
            [0, "Branch shot", "shot", "blue", {"tick": 0}],
            [1, "", "kick_trace", "blue", {"tick": 1, "kick_tick": 0}],
        ],
    )
    pending = timeline.branch_replay(parent, {"fork_tick": 1})
    assert [item[1] for item in pending["events"]] == ["Shared"]
    assert pending["frames"][-1]["stage"] == "Branch point"
    branch = timeline.branch_replay(parent, {"fork_tick": 1}, trial)
    assert branch["positions"] == [[0], [1], [3]]
    assert [frame["t"] for frame in branch["frames"]] == [0, 0.04, 0.08]
    assert [item[1] for item in branch["events"]] == ["Shared", "Branch shot", ""]
    assert branch["events"][-1][4] == {"tick": 2, "kick_tick": 1}
    assert parent["events"][-1][1] == "Original pass"
    assert parent["positions"] == [[0], [1], [2]]


def test_hoca_remembers_one_candidate_per_recording(change_tool, tmp_path, monkeypatch):
    timeline = importlib.import_module("decision_timeline")
    request = request_value()
    request["evidence"]["source"]["seed"] = 123
    saved = tmp_path / "request.json"
    saved.write_text(json.dumps(request))
    monkeypatch.setattr(timeline, "create_candidate", lambda value: {
        "branch": "candidate/example", "record": str(tmp_path / "candidate.json"), "created": True,
    })
    source = "/matches/123456789abc/"
    log = {"seed": 123, "frames": [{}] * 100, "dt": 0.04}
    record, created = timeline.DecisionTimelines(tmp_path, None).create(source, request, log)
    assert created
    assert record["fork_time"] == 3.6
    reopened = timeline.DecisionTimelines(tmp_path, None)
    assert reopened.read(source) == record
    assert reopened.create(source, request, log) == (record, False)
    with pytest.raises(ValueError, match="already has a candidate"):
        reopened.create(source, dict(request, title="Another request"), log)
