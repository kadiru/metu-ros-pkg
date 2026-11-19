"""Unfiltered native self-play movie: goals and defensive stops are both outcomes."""

from pathlib import Path
import time
from roboturk.defended import DEFENDER
from roboturk.keeper import KEEPER
from roboturk.team import evaluate_team
from opponent_demo import Recorder as OpponentRecorder


class Recorder(OpponentRecorder):
    def begin_episode(self, index, seed, env):
        super().begin_episode(index, seed, env)
        self.label(0, "RoboTurk | Mobile defense + alternating self-play", 0.025, 0.05)
        self.label(
            5,
            "Four learned actors | scripted attacking targets / handoff | native Webots",
            0.95,
            0.03,
        )

    def camera(self, env, shot):
        if shot is None:
            return super().camera(env, shot)
        robots = [env.world.robot(k) for k in (env.receiver, DEFENDER, KEEPER)]
        gx = env.world.attack_sign * env.cfg.length / 2
        xs, ys = [r.x for r in robots] + [gx], [r.y for r in robots] + [0]
        x, y = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        z = max(5.5, (max(xs) - min(xs) + 1) * 2.35, (max(ys) - min(ys) + 1) * 1.6)
        self.supervisor.getFromDef("CAMERA").getField("position").setSFVec3f([x, y, z])

    def step(self, env, info, shot):
        super().step(env, info, shot)
        if info["reason"] in ("defender_interception", "keeper_save"):
            total = env.steps * env.cfg.dt + (info["elapsed_seconds"] if shot is not None else 0)
            label = (
                "DEFENDER INTERCEPT" if info["reason"] == "defender_interception" else "KEEPER SAVE"
            )
            self.label(2, f"{label} | defense wins | {total:.2f} s", 0.16, 0.042, 0xFFD660)
        elif info["reason"] not in ("running", "reset", "received", "goal"):
            self.label(2, "CONTROL FAILURE: " + info["reason"], 0.16, 0.042, 0xFF6666)
        ev = env.backend.evidence()
        self.label(
            6,
            f"Defender x travel {ev['defender_x_excursion']:.2f}m | keeper x travel {ev['keeper_x_excursion']:.2f}m",
            0.275,
            0.03,
        )


def record(supervisor, env, transfer, finish, output, episodes=8, seed=2400000):
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    recorder = Recorder(supervisor, output, transfer, finish)
    supervisor.movieStartRecording(str(output.with_suffix(".mp4")), 1280, 720, 0, 85, 1, False)
    result = evaluate_team(env, transfer, episodes, seed, finish, recorder)
    supervisor.movieStopRecording()
    deadline = time.monotonic() + 60
    while not supervisor.movieIsReady() and time.monotonic() < deadline:
        if supervisor.step(20) == -1:
            raise RuntimeError("Webots closed during encoding")
    if not supervisor.movieIsReady() or supervisor.movieFailed():
        raise RuntimeError("Self-play movie encoding failed")
    result.update(
        source="actual Webots movie recorder",
        policy_fallback=False,
        scripted_target_selection=True,
        scripted_role_handoff=True,
        outcome_display_seconds=0.6,
        possession_sample_seconds=0.005,
    )
    return result
