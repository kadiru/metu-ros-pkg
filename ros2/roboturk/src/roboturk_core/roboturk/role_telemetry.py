"""Compact match telemetry shared by the Webots stream and replay exporter."""

import json

from .match_limits import TEAMS, limit_values

STAGES = {
    "NORMAL_FIRST_HALF_PRE": "First kickoff",
    "NORMAL_FIRST_HALF": "First half",
    "NORMAL_HALF_TIME": "Halftime",
    "NORMAL_SECOND_HALF_PRE": "Second kickoff",
    "NORMAL_SECOND_HALF": "Second half",
    "POST_GAME": "Full time",
}
STARTS = {"kickoff_kick": "kicks off", "shot": "shoots"}


def viewer_header(keys, seed, cfg, limits=None):
    return dict(
        keys=keys, seed=seed, dt=cfg.dt, limits=limits or dict.fromkeys(TEAMS, limit_values(cfg))
    )


def compact_frame(frame, tick, dt, sign):
    motions = frame.get("motion")
    return dict(
        t=frame.get("clock", round(tick * dt, 4)),
        score=frame["score"],
        roles=frame["roles"],
        phases=frame["phases"],
        stage=STAGES.get(frame.get("stage"), ""),
        left=frame.get("stage_seconds"),
        command=frame.get("ssl_command", ""),
        mode=frame.get("mode", "running"),
        owner=frame.get("control"),
        sign=frame.get("attack_sign", sign),
        allocation=frame.get("allocation"),
        plays=frame.get("plays"),
        limits=frame.get("limits"),
        motion=[m.get("mode") if m else None for m in motions] if motions else None,
    )


def frame_events(previous, current, tick):
    if previous and previous["score"] != current["score"]:
        blue, yellow = current["score"]
        team = "blue" if blue > previous["score"][0] else "yellow"
        yield [tick, f"{team.title()} goal · {blue}–{yellow}", "goal", team]
    stage = current.get("stage")
    if stage in STAGES and (not previous or stage != previous.get("stage")):
        yield [tick, STAGES[stage], "period", None]
    command = current.get("ssl_command", "")
    if command.startswith("DIRECT_FREE_") and command != (previous or {}).get("ssl_command"):
        team = command.rsplit("_", 1)[-1].lower()
        yield [tick, f"{team.title()} free kick", "restart", team]


def action_events(events, keys):
    def name(index):
        team, number = keys[index]
        return f"{team[0].upper()}{number}"

    for event in events:
        tick, kind = event["tick"], event["event"]
        if kind in ("pass", "restart_pass"):
            actor, receiver = event["actor"], event["receiver"]
            yield [tick, f"{name(actor)} passes to {name(receiver)}", "pass", keys[actor][0]]
        elif kind in STARTS:
            actor = event["actor"]
            speed = f" · {event['speed']:g} m/s" if kind == "shot" and "speed" in event else ""
            yield [tick, f"{name(actor)} {STARTS[kind]}{speed}", kind.split("_")[0], keys[actor][0]]
        elif kind == "referee_placement" and event.get("team"):
            team = event["team"].lower()
            yield [tick, f"{team.title()} ball placed", "restart", team]


class RoleTelemetry:
    def __init__(self, stream, keys, seed, cfg, limits=None):
        self.stream = stream
        self.keys = keys
        self.dt = cfg.dt
        self.previous = None
        self.event_index = 0
        self.write(dict(header=viewer_header(keys, seed, cfg, limits)))

    def write(self, entry):
        if self.stream is None:
            return
        try:
            self.stream.write(json.dumps(entry, separators=(",", ":"), allow_nan=False) + "\n")
            self.stream.flush()
        except OSError as error:
            print(f"Match viewer telemetry stopped: {error}", flush=True)
            self.stream = None

    def sample(self, tick, frame, snapshot, events, sign):
        if self.stream is None:
            return
        timeline = list(frame_events(self.previous, frame, tick))
        timeline.extend(action_events(events[self.event_index :], self.keys))
        self.event_index = len(events)
        self.previous = frame
        self.write(
            dict(
                frame=compact_frame(frame, tick, self.dt, sign),
                position=[round(float(value), 4) for value in (*snapshot[:2], *snapshot[4:])],
                events=timeline,
            )
        )

    def finish(self):
        self.write(dict(done=True))
