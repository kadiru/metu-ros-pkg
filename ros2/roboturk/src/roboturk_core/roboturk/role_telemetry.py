"""Compact match telemetry shared by the Webots stream and replay exporter."""

import json
import math

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
NORMAL_GOAL_HEIGHT = 0.15


def xyz(ball):
    return [ball.x, ball.y, ball.z]


def ball_speed(ball):
    return math.hypot(ball.vx, ball.vy, ball.vz)


class KickTrace:
    """Observe one kick until the ball crosses a line, stops, or is deflected."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.kick = None

    def start(self, event, state, events):
        self.finish(event["tick"], state.ball, events)
        self.kick = event
        self.start_time = state.time
        self.previous = state.ball
        self.peak = state.ball.z

    def sample(self, tick, sample, events):
        if self.kick is None or sample.time <= self.start_time:
            return
        b, p = sample.ball, self.previous
        self.peak = max(self.peak, b.z)
        self.previous = b
        crossings = []
        for axis, limit in (("x", self.cfg.ball_exit_x), ("y", self.cfg.ball_exit_y)):
            last, now = getattr(p, axis), getattr(b, axis)
            if abs(last) < limit <= abs(now):
                crossings.append(((math.copysign(limit, now) - last) / (now - last), axis))
        if crossings:
            fraction, axis = min(crossings)
            point = [a + fraction * (z - a) for a, z in zip(xyz(p), xyz(b), strict=True)]
            mouth = axis == "x" and abs(point[1]) < self.cfg.goal_width / 2 - self.cfg.ball_radius
            outcome = "goal mouth" if mouth else "wide" if axis == "x" else "touchline"
            self.finish(
                tick,
                b,
                events,
                outcome,
                point,
                ball_speed(p) + fraction * (ball_speed(b) - ball_speed(p)),
            )
        elif math.hypot(b.vx - p.vx, b.vy - p.vy) > 0.65 and sample.time - self.start_time > 0.012:
            self.finish(tick, b, events, "deflected")
        elif sample.time - self.start_time >= 2 or ball_speed(b) < 0.13:
            self.finish(tick, b, events)

    def finish(self, tick, ball, events, outcome="ended", point=None, speed=None):
        if self.kick is not None:
            events.append(
                dict(
                    event="kick_trace",
                    tick=tick,
                    kick_tick=self.kick["tick"],
                    actor=self.kick["actor"],
                    point=point if point is not None else xyz(ball),
                    speed=speed if speed is not None else ball_speed(ball),
                    outcome=outcome,
                    peak_z=self.peak,
                )
            )
            self.kick = None


def viewer_header(keys, seed, cfg, limits=None):
    return dict(
        keys=keys, seed=seed, dt=cfg.dt, limits=limits or dict.fromkeys(TEAMS, limit_values(cfg))
    )


def compact_frame(frame, tick, dt, sign, *, ball_vxy=None):
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
        restart=frame.get("restart"),
        owner=frame.get("control"),
        sign=frame.get("attack_sign", sign),
        allocation=frame.get("allocation"),
        plays=frame.get("plays"),
        limits=frame.get("limits"),
        motion=[m.get("mode") if m else None for m in motions] if motions else None,
        ball_z=frame.get("ball_z"),
        ball_vxy=[round(float(value), 4) for value in ball_vxy] if ball_vxy is not None else None,
        ball_vz=frame.get("ball_vz"),
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
        if kind == "kick_trace":
            yield [tick, "", kind, keys[event["actor"]][0], event]
            continue
        details = [event] if "origin" in event else []
        if event.get("spin_turn") and kind in ("pass", "shot"):
            actor = event["actor"]
            move = "Reverse spin" if event["spin_turn"]["reverse"] else "Spin"
            action = f"pass to {name(event['receiver'])}" if kind == "pass" else "shot"
            label = f"{name(actor)} · {move.lower()} {action} · {event['speed']:.1f} m/s"
            yield [tick, label, kind, keys[actor][0], *details]
        elif kind == "pass" and event.get("elevation", 0) > 0:
            actor, receiver = event["actor"], event["receiver"]
            label = f"{name(actor)} chips to {name(receiver)} · {event['speed']:.1f} m/s"
            yield [tick, label, "chip", keys[actor][0], *details]
        elif kind in ("pass", "restart_pass"):
            actor, receiver = event["actor"], event["receiver"]
            yield [
                tick,
                f"{name(actor)} passes to {name(receiver)}",
                "pass",
                keys[actor][0],
                *details,
            ]
        elif kind in STARTS:
            actor = event["actor"]
            speed = f" · {event['speed']:g} m/s" if kind == "shot" and "speed" in event else ""
            recipient = f" to {name(event['receiver'])}" if "receiver" in event else ""
            yield [
                tick,
                f"{name(actor)} {STARTS[kind]}{recipient}{speed}",
                kind.split("_")[0],
                keys[actor][0],
                *details,
            ]
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
                frame=compact_frame(frame, tick, self.dt, sign, ball_vxy=snapshot[2:4]),
                position=[round(float(value), 4) for value in (*snapshot[:2], *snapshot[4:])],
                events=timeline,
            )
        )

    def finish(self):
        self.write(dict(done=True))
