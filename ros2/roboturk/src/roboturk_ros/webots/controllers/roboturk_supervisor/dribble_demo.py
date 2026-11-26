"""Record consecutive actual Webots episodes controlled only by the learned policy."""

import math
from pathlib import Path
import time
from roboturk.dribble import evaluate_dribble
from roboturk.model import possession


class Recorder:
    def __init__(self, supervisor, output):
        self.supervisor = supervisor
        self.output = Path(output)
        self.preview_saved = False
        supervisor.getRoot().getField("children").importMFNodeFromString(
            -1,
            """
            DEF DRIBBLE_TARGET Pose {
              translation 3.6 0 0.004
              children [ Shape {
                appearance PBRAppearance { baseColor 0.2 0.85 1 transparency 0.35 roughness 1 metalness 0 }
                geometry Box { size 0.36 0.36 0.003 }
              } ]
            }
        """,
        )

    def label(self, index, text, y, size=0.042, color=0xFFFFFF):
        self.supervisor.setLabel(index, text, 0.025, y, size, color, 0, "Arial")

    def begin_episode(self, index, seed, env):
        r, target = env.world.robot(env.key), env.intent.target
        self.follow = index % 2 == 1
        # Keep the route and actual goal in view; only this presentation camera moves.
        self.supervisor.getFromDef("CAMERA").getField("position").setSFVec3f(
            [(r.x + target[0]) / 2, 0, max(9.0, (abs(target[0] - r.x) + 2.5) * 2.2)]
        )
        self.supervisor.getFromDef("DRIBBLE_TARGET").getField("translation").setSFVec3f(
            [*target, 0.004]
        )
        self.label(0, "RoboTurk | Learned capture + dribble", 0.025, 0.06)
        kind = "rolling" if env.scenario["moving"] else "stationary"
        side = "+X" if env.world.attack_sign == 1 else "-X"
        view = "tracking view" if self.follow else "route view"
        self.label(
            1, f"Episode {index + 1} | seed {seed} | {kind} ball | goal {side} | {view}", 0.10, 0.04
        )
        self.label(
            4, "Neural control: demonstrations + PPO | approximate Webots dynamics", 0.93, 0.034
        )

    def step(self, env, info):
        if self.follow:
            robot = env.world.robot(env.key)
            self.supervisor.getFromDef("CAMERA").getField("position").setSFVec3f(
                [robot.x + 0.25 * env.world.attack_sign, robot.y, 4.0]
            )
        phase = (
            "CONTROLLED ARRIVAL"
            if info["is_success"]
            else "DRIBBLING"
            if env.acquired
            else "CAPTURING"
        )
        color = 0x68FFB0 if env.acquired else 0xFFFFFF
        self.label(
            2,
            f"{phase} | {env.steps * env.cfg.dt:.2f} s | to target {info['target_distance']:.2f} m",
            0.16,
            color=color,
        )
        self.label(
            3,
            f"Carry progress {info['carried_progress']:.2f} m | losses {info['lost_possession_checks']} | arrival hold {env.arrival_dwell}/50",
            0.215,
            0.038,
        )
        if (
            not self.preview_saved
            and self.follow
            and env.acquired
            and info["target_distance"] < 0.7
        ):
            self.supervisor.exportImage(str(self.output.with_suffix(".jpg")), 95)
            self.preview_saved = True


def record(supervisor, env, policy, output, episodes=6, seed=160000):
    output = Path(output)
    recorder = Recorder(supervisor, output)
    supervisor.movieStartRecording(str(output.with_suffix(".mp4")), 1280, 720, 0, 85, 1, False)
    records = []
    for i in range(episodes):

        class EpisodeRecorder:
            def begin_episode(self, _, actual_seed, current):
                recorder.begin_episode(i, actual_seed, current)

            def step(self, current, info):
                recorder.step(current, info)

        result = evaluate_dribble(env, policy, 1, seed + i, EpisodeRecorder())
        records.extend(result["episodes"])
        retained_checks = total_checks = 0
        for _ in range(40):
            command = policy.command(env.world, env.key, env.intent, env.cfg)
            env.world = env.backend.step({env.key: command})
            for w in env.backend.last_substep_worlds:
                total_checks += 1
                retained_checks += int(possession(w, env.key, env.cfg) and command.dribbler > 0.5)
            r, b = env.world.robot(env.key), env.world.ball
            if not all(math.isfinite(v) for v in [r.x, r.y, b.x, b.y]):
                raise ValueError("Nonfinite demo state")
        records[-1]["extra_retention_checks"] = retained_checks
        records[-1]["extra_total_checks"] = total_checks
    supervisor.movieStopRecording()
    deadline = time.monotonic() + 60
    while not supervisor.movieIsReady() and time.monotonic() < deadline:
        if supervisor.step(20) == -1:
            raise RuntimeError("Webots closed during movie encoding")
    if not supervisor.movieIsReady() or supervisor.movieFailed():
        raise RuntimeError("Webots movie encoding failed or timed out")
    return {
        "episodes": records,
        "source": "actual Webots movie recorder",
        "policy_fallback": False,
        "extra_retention_seconds": 0.8,
        "possession_sample_seconds": 0.005,
    }
