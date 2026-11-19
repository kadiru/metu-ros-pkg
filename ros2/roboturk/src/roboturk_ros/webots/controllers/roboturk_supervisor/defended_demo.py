"""Native Webots recording with all three robots and measured lane clearance."""

from pathlib import Path
import time
from roboturk.defended import DEFENDER, lane_clearance
from roboturk.team import evaluate_team
from team_demo import Recorder as TeamRecorder


class Recorder(TeamRecorder):
    def begin_episode(self, index, seed, env):
        super().begin_episode(index, seed, env)
        self.label(0, "RoboTurk | Pass around a moving defender", 0.025, 0.055)
        self.label(
            4,
            "BLUE: passer   CYAN: receiver / scorer   YELLOW: reactive lane defender",
            0.90,
            0.035,
        )
        self.label(
            5,
            f"Receiving-point tactic + learned controls | guard {env.backend.defender.speed:.2f} m/s | approximate Webots physics",
            0.95,
            0.03,
        )
        env.backend.nodes[DEFENDER].getField("color").setSFColor([0.95, 0.8, 0.05])

    def camera(self, env, shot):
        if shot is not None:
            return super().camera(env, shot)
        robots = env.world.robots
        xs, ys = [r.x for r in robots], [r.y for r in robots]
        z = max(9.0, (max(xs) - min(xs) + 1.2) * 2.7, (max(ys) - min(ys) + 1.2) * 1.6)
        self.supervisor.getFromDef("CAMERA").getField("position").setSFVec3f(
            [(min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, z]
        )

    def step(self, env, info, shot):
        super().step(env, info, shot)
        if shot is None:
            phase = (
                "RECEIVED"
                if env.state.received
                else "PASS AROUND DEFENDER"
                if env.state.passed
                else "OPENING A PASSING LANE"
            )
            if env.finished and not info["is_success"]:
                phase = "FAILED: " + info["reason"]
            self.label(2, f"{phase} | {info['elapsed_seconds']:.2f} s", 0.16, 0.042, 0x68FFB0)
            self.label(
                3,
                f"Lane clearance {lane_clearance(env.world, env.receiver):.2f} m | defender contacts {env.backend.ball_contacts}",
                0.22,
                0.035,
            )


def record(supervisor, env, policy, shooter, output, episodes=6, seed=660000):
    output = Path(output)
    recorder = Recorder(supervisor, output, policy, shooter)
    supervisor.movieStartRecording(str(output.with_suffix(".mp4")), 1280, 720, 0, 85, 1, False)
    result = evaluate_team(env, policy, episodes, seed, shooter, recorder)
    supervisor.movieStopRecording()
    deadline = time.monotonic() + 60
    while not supervisor.movieIsReady() and time.monotonic() < deadline:
        if supervisor.step(20) == -1:
            raise RuntimeError("Webots closed during movie encoding")
    if not supervisor.movieIsReady() or supervisor.movieFailed():
        raise RuntimeError("Webots movie encoding failed or timed out")
    result.update(
        source="actual Webots movie recorder",
        policy_fallback=False,
        scripted_role_handoff=True,
        opponent="reactive_lane_defender_v1",
        outcome_display_seconds=0.6,
        possession_sample_seconds=0.005,
    )
    return result
