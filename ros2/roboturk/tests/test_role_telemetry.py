import io
import json

from roboturk.five_roles import CFG
from roboturk.role_telemetry import RoleTelemetry, compact_frame


def test_stream_uses_replay_frames_and_emits_new_events_once():
    keys = [["blue", 0], ["yellow", 0]]
    sample = [1.25, -0.5, 0.8, 0.2, -4.0, 0.0, 0.1, 4.0, 0.0, 3.0]
    kickoff = dict(score=[0, 0], roles=["Keeper", "Keeper"], phases=["Attack", "Defend"])
    goal = dict(kickoff, score=[1, 0], stage="NORMAL_FIRST_HALF")
    stream = io.StringIO()
    telemetry = RoleTelemetry(stream, keys, 42, CFG)
    events = [dict(tick=0, event="kickoff_kick", actor=0)]
    telemetry.sample(0, kickoff, sample, events, 1)
    telemetry.sample(1, goal, sample, events, 1)
    telemetry.finish()

    header, first, second, finish = map(json.loads, stream.getvalue().splitlines())
    assert header["header"]["keys"] == keys
    assert first["frame"] == compact_frame(kickoff, 0, CFG.dt, 1)
    assert first["position"] == sample[:2] + sample[4:]
    assert first["events"] == [[0, "B0 kicks off", "kickoff", "blue"]]
    assert second["events"] == [
        [1, "Blue goal · 1–0", "goal", "blue"],
        [1, "First half", "period", None],
    ]
    assert finish == {"done": True}
