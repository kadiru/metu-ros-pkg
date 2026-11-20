"""Uncut native continuous matches; every seed, frame and outcome is retained."""

from pathlib import Path
import time
from roboturk.continuous import KEYS, MODES, name
from roboturk.continuous_training import evaluate


class Recorder:
    def __init__(self, supervisor, count):
        self.supervisor, self.count = supervisor, count
        self.event_index, self.event_until, self.event_text = 0, 0, ""

    def label(self, index, text, y, size=0.045, color=0xFFFFFF):
        self.supervisor.setLabel(index, text, 0.025, y, size, color, 0, "Arial")

    def begin(self, index, match):
        self.event_index, self.event_until, self.event_text = 0, 0, ""
        camera = self.supervisor.getFromDef("CAMERA")
        # Fixed whole-field view. Both teammates and both opponents stay in frame.
        camera.getField("position").setSFVec3f([0, 0, 11.5])
        camera.getField("orientation").setSFRotation(
            [-0.577350269, 0.577350269, 0.577350269, 2.094395102]
        )
        self.label(0, "RoboTurk | Continuous 2v2 self-play", 0.025, 0.065)
        self.label(
            1,
            f"Match {index + 1}/{self.count} | seed {match.seed} | Blue attacks {'right' if match.sign == 1 else 'left'}",
            0.10,
            0.043,
        )
        colors = ([0.05, 0.25, 0.95], [0.05, 0.85, 0.85], [0.95, 0.8, 0.05], [0.95, 0.35, 0.05])
        for k, color in zip(KEYS, colors, strict=True):
            match.backend.nodes[k].getField("color").setSFColor(color)
        self.label(
            7,
            "Native Webots | neural behavior + motion | simplified referee restarts | approximate physics",
            0.95,
            0.032,
        )
        self.step(match)

    def step(self, match):
        remaining = max(0, match.duration - match.steps * match.cfg.dt)
        self.label(
            2,
            f"BLUE {match.scores['blue']}  :  {match.scores['yellow']} YELLOW     {remaining:05.2f}s remaining",
            0.16,
            0.055,
        )
        for i, team in enumerate(("blue", "yellow")):
            text = "   ".join(
                f"{name(k)}: {MODES[match.memory.mode[k]]}" for k in KEYS if k.team == team
            )
            self.label(
                3 + i, text, 0.84 + i * 0.05, 0.038, 0x6FE7FF if team == "blue" else 0xFFE060
            )
        for event in match.events[self.event_index :]:
            typ = event["type"]
            if typ in ("goal", "turnover", "reception", "referee_restart", "ball_out"):
                self.event_text = (
                    typ.replace("_", " ").upper() + " " + event.get("team", event.get("reason", ""))
                )
                self.event_until = match.steps + 60
        self.event_index = len(match.events)
        self.label(
            5, self.event_text if match.steps < self.event_until else "", 0.22, 0.044, 0xA4FFAB
        )


def record(supervisor, backend, blue, yellow, output, count, seed, duration):
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    recorder = Recorder(supervisor, count)
    supervisor.movieStartRecording(str(output.with_suffix(".mp4")), 1280, 720, 0, 85, 1, False)
    result = evaluate(backend, blue, yellow, count, seed, duration, recorder=recorder)
    supervisor.movieStopRecording()
    deadline = time.monotonic() + 60
    while not supervisor.movieIsReady() and time.monotonic() < deadline:
        if supervisor.step(20) == -1:
            raise RuntimeError("Webots closed during movie encoding")
    if not supervisor.movieIsReady() or supervisor.movieFailed():
        raise RuntimeError("Native continuous movie encoding failed")
    result.update(
        source="actual Webots movie recorder",
        speed=1,
        cuts=0,
        outcome_selection=False,
        match_seconds=count * duration,
        reset_settling_seconds=0.005,
        fixed_whole_field_camera=True,
        post_match_display_seconds=0,
    )
    return result
