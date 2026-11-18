"""Native Webots capture/carry/score movie; the policy chooses every motion and shot."""

from pathlib import Path
import time
from roboturk.shoot import evaluate_shoot


class Recorder:
    def __init__(self, supervisor, output):
        self.supervisor, self.output = supervisor, Path(output)
        self.preview_saved = False
        self.follow = False

    def label(self, index, text, y, size=0.042, color=0xFFFFFF):
        self.supervisor.setLabel(index, text, 0.025, y, size, color, 0, "Arial")

    def begin_episode(self, index, seed, env):
        self.follow = index % 2 == 1
        r, target = env.world.robot(env.key), env.intent.target
        self.supervisor.getFromDef("CAMERA").getField("position").setSFVec3f(
            [(r.x + target[0]) / 2, 0, max(9.0, (abs(target[0] - r.x) + 2.0) * 2.2)]
        )
        kind = "rolling" if env.scenario["moving"] else "stationary"
        side = "+X" if env.world.attack_sign == 1 else "-X"
        view = "tracking view" if self.follow else "route view"
        self.label(0, "RoboTurk | Learned capture, dribble + score", 0.025, 0.06)
        self.label(
            1, f"Episode {index + 1} | seed {seed} | {kind} ball | goal {side} | {view}", 0.10, 0.04
        )
        self.label(
            4,
            "Neural motion + fire request | fixed 3.5 m/s kick | approximate Webots dynamics",
            0.93,
            0.033,
        )

    def step(self, env, info):
        if self.follow:
            r = env.world.robot(env.key)
            # Before shooting, follow the robot; afterward show the ball's actual flight and goal.
            x = (
                (env.world.ball.x + env.intent.target[0]) / 2
                if env.shot
                else r.x + 0.25 * env.world.attack_sign
            )
            self.supervisor.getFromDef("CAMERA").getField("position").setSFVec3f(
                [x, 0 if env.shot else r.y, 5.5 if env.shot else 4.0]
            )
        phase = (
            "GOAL"
            if info["is_success"]
            else "SHOT"
            if env.shot
            else "DRIBBLING"
            if env.acquired
            else "CAPTURING"
        )
        if env.finished and not info["is_success"]:
            phase = "FAILED: " + info["reason"]
        self.label(
            2,
            f"{phase} | {info['elapsed_seconds']:.2f} s | goal distance {info['target_distance']:.2f} m",
            0.16,
            color=0x68FFB0 if env.acquired else 0xFFFFFF,
        )
        self.label(
            3,
            f"Carry progress {info['carried_progress']:.2f} m | pre-shot losses {info['lost_possession_checks']} | kicks {info['applied_kicks']}",
            0.215,
            0.038,
        )
        if self.follow and env.shot and not self.preview_saved:
            self.supervisor.exportImage(str(self.output.with_suffix(".jpg")), 95)
            self.preview_saved = True


def record(supervisor, env, policy, output, episodes=6, seed=260000):
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

        result = evaluate_shoot(env, policy, 1, seed + i, EpisodeRecorder())
        records.extend(result["episodes"])
        # Keep showing the measured outcome while the same neural policy continues for 0.6 s.
        for _ in range(30):
            command = policy.command(env.world, env.key, env.intent, env.cfg)
            env.world = env.backend.step({env.key: command})
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
        "outcome_display_seconds": 0.6,
        "possession_sample_seconds": 0.005,
    }
