"""Native Webots movie of consecutive, unfiltered learned two-robot episodes."""

from pathlib import Path
import time
from roboturk.model import Command, Intent, Skill
from roboturk.team import evaluate_team
from roboturk.team_contract import RECEIVER_CONTRACT


class Recorder:
    def __init__(self, supervisor, output, policy, shooter):
        self.supervisor, self.output = supervisor, Path(output)
        self.policy, self.shooter = policy, shooter
        self.preview_saved = False

    def label(self, index, text, y, size=0.04, color=0xFFFFFF):
        self.supervisor.setLabel(index, text, 0.025, y, size, color, 0, "Arial")

    def begin_episode(self, index, seed, env):
        self.label(0, "RoboTurk | Learned pass, receive + score", 0.025, 0.06)
        self.label(
            1,
            f"Episode {index + 1} | seed {seed} | goal {'+X' if env.world.attack_sign == 1 else '-X'}",
            0.10,
        )
        self.label(
            4,
            f"BLUE: passer #{env.passer.id}    CYAN: receiver / scorer #{env.receiver.id}   |   Both are teammates",
            0.90,
            0.035,
        )
        self.label(
            5,
            (
                "Fitted close-range receiver"
                if self.policy.contract == RECEIVER_CONTRACT
                else "Neural transfer"
            )
            + " | frozen neural scorer | scripted role handoff | Webots",
            0.95,
            0.03,
        )
        env.backend.nodes[env.passer].getField("color").setSFColor([0.05, 0.25, 0.95])
        env.backend.nodes[env.receiver].getField("color").setSFColor([0.05, 0.85, 0.85])
        self.camera(env, None)

    def camera(self, env, shot):
        p, r = env.world.robot(env.passer), env.world.robot(env.receiver)
        if shot is None:
            x, y, z = (p.x + r.x) / 2, (p.y + r.y) / 2, max(7.5, (abs(p.x - r.x) + 1.5) * 2.3)
        elif shot.shot:
            x, y, z = (r.x + shot.intent.target[0]) / 2, 0, 6.5
        else:
            x, y, z = r.x + env.world.attack_sign * 0.25, r.y, 5.5
        self.supervisor.getFromDef("CAMERA").getField("position").setSFVec3f([x, y, z])

    def step(self, env, info, shot):
        self.camera(env, shot)
        if shot is None:
            phase = (
                "RECEIVED"
                if env.state.received
                else "PASS IN FLIGHT"
                if env.state.passed
                else "ALIGNING PASS"
            )
            total = info["elapsed_seconds"]
            detail = f"Pass distance {(env.state.pass_distance or 0):.2f} m | reception hold {env.state.receiver_dwell * env.cfg.dt:.2f} s"
        else:
            phase = "GOAL" if info["is_success"] else "SHOT" if shot.shot else "RECEIVER DRIBBLING"
            total = env.steps * env.cfg.dt + info["elapsed_seconds"]
            detail = f"Controlled carry {info['carried_progress']:.2f} m | goal distance {info['target_distance']:.2f} m"
        if (env.finished if shot is None else shot.finished) and not info["is_success"]:
            phase = "FAILED: " + info["reason"]
        self.label(2, f"{phase} | {total:.2f} s", 0.16, 0.045, 0x68FFB0)
        self.label(3, detail, 0.22, 0.035)
        if env.state.passed and not env.state.received and not self.preview_saved:
            # Wait until the free ball has travelled visibly between teammates.
            b = env.world.ball
            if (b.x - env.state.pass_x) ** 2 + (b.y - env.state.pass_y) ** 2 > 0.35**2:
                self.supervisor.exportImage(str(self.output.with_suffix(".jpg")), 95)
                self.preview_saved = True

    def end_episode(self, env, result):
        for _ in range(30):
            if env.state.received:
                goal = (env.world.attack_sign * env.cfg.length / 2, 0)
                commands = {
                    env.passer: Command(),
                    env.receiver: self.shooter.command(
                        env.world, env.receiver, Intent(Skill.SHOOT, goal), env.cfg
                    ),
                }
            else:
                commands = env.decode_action(self.policy.action(env))
            env.world = env.backend.step(commands)


def record(supervisor, env, policy, shooter, output, episodes=6, seed=460000):
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
        outcome_display_seconds=0.6,
        possession_sample_seconds=supervisor.getBasicTimeStep() / 1000,
    )
    return result
