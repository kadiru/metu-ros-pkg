"""Native recordings of consecutive independently qualified defensive micro tasks."""

from pathlib import Path
import time
from roboturk.defense_micro import GUARD, TARGET
from roboturk.model import Command
from roboturk.micro_training import evaluate


class Recorder:
    def __init__(self, supervisor):
        self.supervisor = supervisor

    def label(self, index, text, y, size=0.04):
        self.supervisor.setLabel(index, text, 0.025, y, size, 0xFFFFFF, 0, "Arial")

    def begin_episode(self, index, seed, env):
        title = (
            "Learned shot block / retain"
            if env.kind == "block"
            else "Learned passing-lane coverage"
        )
        self.label(0, "RoboTurk | " + title, 0.025, 0.052)
        self.label(1, f"Episode {index + 1} | seed {seed}", 0.10)
        self.label(4, "CYAN: learned defender | fixed line / target-selection tactic", 0.9, 0.033)
        self.label(
            5, "Separate micro curriculum | actual Webots physics | no teacher fallback", 0.95, 0.03
        )
        env.backend.nodes[GUARD].getField("color").setSFColor([0.05, 0.85, 0.85])
        if env.kind == "cover":
            env.backend.nodes[TARGET].getField("color").setSFColor([0.95, 0.15, 0.12])
        r, b = env.world.robot(GUARD), env.world.ball
        x = (r.x + b.x) / 2 if env.kind == "block" else 0
        self.supervisor.getFromDef("CAMERA").getField("position").setSFVec3f(
            [x, 0, 7.5 if env.kind == "block" else 9.0]
        )

    def step(self, env, info):
        phase = (
            ("SAVED / RETAINED" if env.kind == "block" else "LANE COVERED")
            if info["is_success"]
            else info["reason"].upper()
        )
        self.label(2, f"{phase} | {info['elapsed_seconds']:.2f} s", 0.16, 0.042)
        self.label(
            3, f"Continuous measured dwell: {info['stable_dwell_seconds']:.2f} s", 0.22, 0.035
        )

    def end_episode(self, env):
        for _ in range(20):
            env.world = env.backend.step({GUARD: Command(dribbler=1)})


def record(supervisor, env, actor, output, episodes, seed):
    supervisor.movieStartRecording(
        str(Path(output).with_suffix(".mp4")), 1280, 720, 0, 85, 1, False
    )
    result = evaluate(env, actor, episodes, seed, Recorder(supervisor))
    supervisor.movieStopRecording()
    deadline = time.monotonic() + 60
    while not supervisor.movieIsReady() and time.monotonic() < deadline:
        if supervisor.step(20) == -1:
            raise RuntimeError("Webots closed while encoding micro demo")
    if not supervisor.movieIsReady() or supervisor.movieFailed():
        raise RuntimeError("Micro movie encoding failed")
    result.update(
        source="actual Webots movie recorder", policy_fallback=False, outcome_display_seconds=0.4
    )
    return result
