"""Record actual Webots frames while a checkpoint runs, without teacher fallback."""

import json
import math
from pathlib import Path
import time
from roboturk.capture import evaluate_capture
from roboturk.capture_contract import capture_observation
from roboturk.model import possession


class Recorder:
    def __init__(self, supervisor, output):
        self.supervisor = supervisor
        self.output = Path(output)
        self.preview_saved = False
        self.episode = 0

    def begin_episode(self, index, seed, env):
        self.episode = index
        r, b = env.world.robot(env.key), env.world.ball
        camera = self.supervisor.getFromDef("CAMERA")
        distance = math.hypot(r.x - b.x, r.y - b.y)
        camera.getField("position").setSFVec3f(
            [(r.x + b.x) / 2, (r.y + b.y) / 2, max(3.6, 2.2 * distance + 1)]
        )
        self.supervisor.setLabel(
            0, "RoboTurk | Learned chase & capture", 0.025, 0.025, 0.045, 0xFFFFFF, 0, "Arial"
        )
        kind = "rolling ball" if env.scenario["moving"] else "stationary ball"
        self.supervisor.setLabel(
            1,
            f"Episode {index + 1} | seed {seed} | {kind}",
            0.025,
            0.10,
            0.032,
            0xFFFFFF,
            0,
            "Arial",
        )
        self.supervisor.setLabel(
            3,
            "Neural policy: demonstrations + PPO | approximate Webots dynamics",
            0.025,
            0.93,
            0.026,
            0xFFFFFF,
            0,
            "Arial",
        )

    def step(self, env, info):
        status = "CAPTURED" if info["is_success"] else f"Chasing | hold {env.dwell}/20"
        self.supervisor.setLabel(
            2,
            f"{status} | {env.steps * env.cfg.dt:.2f} s",
            0.025,
            0.155,
            0.034,
            0x68FFB0 if info["is_success"] else 0xFFFFFF,
            0,
            "Arial",
        )
        if not self.preview_saved and env.steps >= 25:
            self.supervisor.exportImage(str(self.output.with_suffix(".jpg")), 95)
            self.preview_saved = True


def record(supervisor, env, policy, output, episodes=6, seed=60000):
    output = Path(output)
    recorder = Recorder(supervisor, output)
    supervisor.movieStartRecording(str(output.with_suffix(".mp4")), 1280, 720, 0, 85, 1, False)
    records = []
    for i in range(episodes):
        # Use explicit consecutive seeds. Do not discard failed episodes from the video.
        class EpisodeRecorder:
            def begin_episode(self, _, actual_seed, current_env):
                recorder.begin_episode(i, actual_seed, current_env)

            def step(self, current_env, info):
                recorder.step(current_env, info)

        result = evaluate_capture(env, policy, 1, seed + i, EpisodeRecorder())
        records.extend(result["episodes"])
        # Continue the same neural policy for a further second to show retention after success.
        retained = 0
        for _ in range(50):
            command = policy.command(env.world, env.key, env.intent, env.cfg)
            env.world = env.backend.step({env.key: command})
            capture_observation(env.world, env.key, env.cfg)  # reject nonfinite state
            r, b = env.world.robot(env.key), env.world.ball
            held = (
                possession(env.world, env.key, env.cfg)
                and math.hypot(r.vx - b.vx, r.vy - b.vy) < 0.2
                and command.dribbler > 0.5
            )
            retained += int(held)
            supervisor.setLabel(
                2,
                "CAPTURED | retaining ball" if held else "Retention lost",
                0.025,
                0.155,
                0.034,
                0x68FFB0 if held else 0xFF7777,
                0,
                "Arial",
            )
        records[-1]["retained_frames_after_capture"] = retained
        records[-1]["retention_frames"] = 50
    supervisor.movieStopRecording()
    deadline = time.monotonic() + 60
    while not supervisor.movieIsReady() and time.monotonic() < deadline:
        if supervisor.step(20) == -1:
            raise RuntimeError("Webots closed during movie encoding")
    if not supervisor.movieIsReady() or supervisor.movieFailed():
        raise RuntimeError("Webots movie encoding failed or timed out")
    result = {
        "episodes": records,
        "source": "actual Webots movie recorder",
        "policy_fallback": False,
        "post_capture_neural_retention_seconds": 1.0,
    }
    output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
    return result
