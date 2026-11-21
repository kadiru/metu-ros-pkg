"""Native recorder keeps every predetermined skill case, including failures."""

from pathlib import Path
import time
from roboturk.model import Command


class Recorder:
    def __init__(self, supervisor, count):
        self.supervisor = supervisor
        self.count = count

    def label(self, index, text, y, size=0.045, color=0xFFFFFF):
        self.supervisor.setLabel(index, text, 0.025, y, size * 1.3, color, 0, "Arial")

    def begin(self, index, env):
        camera = self.supervisor.getFromDef("CAMERA")
        camera.getField("position").setSFVec3f([0, 0, 16.5])
        camera.getField("orientation").setSFRotation(
            [-0.577350269, 0.577350269, 0.577350269, 2.094395102]
        )
        self.label(0, "RoboTurk | TDP-inspired learned skill policies", 0.025, 0.057)
        self.label(
            1,
            f"{env.kind.upper()} | case {index + 1}/{self.count} | seed {env.seed_value}",
            0.105,
            0.043,
        )
        self.label(
            2,
            f"Actor: {env.actor.team}{env.actor.id} | attacks "
            + ("right" if env.sign == 1 else "left"),
            0.17,
            0.037,
        )
        self.label(
            5, "Scripted role/intent | learned motion and release | actual Webots", 0.90, 0.030
        )
        self.label(6, "Approximate physics | fixed kick power | no motion fallback", 0.95, 0.028)
        self.label(3, "", 0.24)
        self.step(env, env.info(False, "running"))

    def step(self, env, info):
        self.label(
            4,
            f"{env.steps * env.cfg.dt:05.2f}s | stable {env.dwell * env.cfg.dt:.2f}s | actor kicks {len(env.applied)}",
            0.82,
            0.036,
        )
        if info["is_success"]:
            self.label(3, "SUCCESS", 0.24, 0.047, 0x88FFAA)
        elif env.finished:
            self.label(3, info["reason"].upper(), 0.24, 0.047, 0xFF8877)

    def end(self, env, info):
        # Visible outcome hold is outside scored episode samples; all robots halt.
        for _ in range(30):
            env.backend.step({r.key: Command() for r in env.world.robots})


def finish_movie(supervisor):
    supervisor.movieStopRecording()
    deadline = time.monotonic() + 60
    while not supervisor.movieIsReady() and time.monotonic() < deadline:
        if supervisor.step(20) == -1:
            raise RuntimeError("Webots ended during encoding")
    if not supervisor.movieIsReady() or supervisor.movieFailed():
        raise RuntimeError("Native movie encoding failed")


def record(supervisor, env, actor, output, count, seed, evaluate):
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    supervisor.movieStartRecording(str(output.with_suffix(".mp4")), 1280, 720, 0, 85, 1, False)
    result = evaluate(env, actor, count, seed, Recorder(supervisor, count))
    finish_movie(supervisor)
    result.update(
        source="actual Webots movie recorder",
        cuts=0,
        outcome_selection=False,
        speed=1,
        post_episode_hold_seconds=0.6,
    )
    return result
