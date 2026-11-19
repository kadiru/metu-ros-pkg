"""Consecutive native Webots demos against frozen learned opponents."""

from pathlib import Path
import time
from roboturk.keeper import KEEPER
from roboturk.defended import DEFENDER
from roboturk.team import evaluate_team
from team_demo import Recorder as TeamRecorder


class Recorder(TeamRecorder):
    def begin_episode(self, index, seed, env):
        super().begin_episode(index, seed, env)
        self.label(0, "RoboTurk | Learned attackers vs learned defense", 0.025, 0.055)
        self.label(
            4, "BLUE: passer   CYAN: receiver   YELLOW: defender   RED: goalkeeper", 0.90, 0.032
        )
        self.label(
            5,
            "Four neural controls | scripted attacking targets / handoff | Webots physics",
            0.95,
            0.03,
        )
        env.backend.nodes[DEFENDER].getField("color").setSFColor([0.95, 0.8, 0.05])
        env.backend.nodes[KEEPER].getField("color").setSFColor([0.95, 0.15, 0.12])

    def camera(self, env, shot):
        r, b = env.world.robot(env.receiver), env.world.ball
        if shot is None:
            # Establish all four roles, then follow the pass at a readable scale.
            robots = tuple(r for r in env.world.robots if env.steps < 40 or r.key != KEEPER)
            xs, ys = [p.x for p in robots], [p.y for p in robots]
            x, y = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
            z = max(10.0, (max(xs) - min(xs) + 1.0) * 2.35, (max(ys) - min(ys) + 1.2) * 1.6)
        else:
            gx = env.world.attack_sign * env.cfg.length / 2
            x, y = (r.x + gx) / 2, b.y / 2
            z = max(5.0, (abs(gx - r.x) + 1.0) * 2.2, (abs(b.y) + 1.0) * 1.6)
        self.supervisor.getFromDef("CAMERA").getField("position").setSFVec3f([x, y, z])

    def step(self, env, info, shot):
        super().step(env, info, shot)
        if shot is not None and not shot.shot:
            phase = "CHANGE AIM / FINISH" if shot.plan.phase else "DRIBBLE / DRAW KEEPER"
            self.label(
                2,
                f"{phase} | {env.steps * env.cfg.dt + info['elapsed_seconds']:.2f} s",
                0.16,
                0.042,
                0x68FFB0,
            )
        self.label(
            6,
            f"Neural defender + keeper | calls {env.backend.defender.calls} / {env.backend.keeper.calls}",
            0.275,
            0.03,
        )


def record(supervisor, env, policy, shooter, output, episodes=6, seed=1660000):
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
        opponent_policy_fallback=False,
        scripted_target_selection=True,
        scripted_role_handoff=True,
        outcome_display_seconds=0.6,
        possession_sample_seconds=0.005,
    )
    return result
