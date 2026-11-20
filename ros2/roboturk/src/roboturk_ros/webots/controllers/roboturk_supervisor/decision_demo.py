"""Every consecutive native game, with the learned decision and targets shown."""

from dataclasses import replace
from pathlib import Path
import time
from roboturk.decision_play import OPTIONS, evaluate
from roboturk.model import Referee
from roboturk.defended import DEFENDER
from roboturk.keeper import KEEPER


class Recorder:
    def __init__(self, supervisor):
        self.supervisor = supervisor

    def label(self, index, text, y, size=0.04, color=0xFFFFFF):
        self.supervisor.setLabel(index, text, 0.025, y, size, color, 0, "Arial")

    def begin_episode(self, index, env):
        self.label(0, "RoboTurk | Learned decisions + self-play", 0.025, 0.05)
        self.label(
            1,
            f"Episode {index + 1} | seed {env.seed} | {env.info()['family']} | goal {'+X' if env.world.attack_sign == 1 else '-X'}",
            0.10,
            0.037,
        )
        self.label(4, "BLUE / CYAN: teammates   YELLOW: defender   RED: keeper", 0.90, 0.032)
        self.label(
            5,
            "Neural behavior, robot, targets and handoff | frozen neural motion skills | native Webots",
            0.95,
            0.029,
        )
        for key, color in [
            (env.keys[0], [0.05, 0.25, 0.95]),
            (env.keys[1], [0.05, 0.85, 0.85]),
            (DEFENDER, [0.95, 0.8, 0.05]),
            (KEEPER, [0.95, 0.15, 0.12]),
        ]:
            env.backend.nodes[key].getField("color").setSFColor(color)
        self.step(env)

    def step(self, env):
        d, w = env.play.decision, env.world
        keys = (
            (*env.keys, DEFENDER, KEEPER)
            if d.option == 0 or env.steps < 40
            else (env.keys[d.owner], DEFENDER, KEEPER)
        )
        points = [(w.robot(k).x, w.robot(k).y) for k in keys] + [(w.ball.x, w.ball.y)]
        if d.option != 0:
            points.append((w.attack_sign * env.cfg.length / 2, 0))
        xs, ys = zip(*points)
        x, y = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        z = max(5.5, (max(xs) - min(xs) + 1) * 2.35, (max(ys) - min(ys) + 1) * 1.6)
        self.supervisor.getFromDef("CAMERA").getField("position").setSFVec3f([x, y, z])
        title = (
            env.reason.upper().replace("_", " ")
            if env.finished
            else f"NEURAL DECISION: {OPTIONS[d.option].upper()} | robot #{env.keys[d.owner].id}"
        )
        color = (
            0x68FFB0
            if not env.finished or env.reason == "goal"
            else 0xFFD660
            if env.reason in ("keeper_save", "defender_interception")
            else 0xFF6666
        )
        self.label(2, f"{title} | {env.steps * env.cfg.dt:.2f}s", 0.16, 0.041, color)
        target = d.receive if d.option == 0 else d.parking if d.option == 1 else d.aim
        self.label(
            3,
            f"Learned target ({target[0]:+.2f}, {target[1]:+.2f})m | carry {max(env.play.carry):.2f}m",
            0.22,
            0.034,
        )
        self.label(
            6,
            f"Manager calls {env.play.calls} | behavior switches {env.option_changes} | robot switches {env.owner_changes}",
            0.275,
            0.030,
        )

    def end_episode(self, env):
        native = env.backend
        while hasattr(native, "base"):
            native = native.base
        native.world = replace(native.world, referee=Referee.HALT)
        for _ in range(30):
            env.world = env.backend.step({})


def record(supervisor, env, output, count=9, seed=3400000):
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    recorder = Recorder(supervisor)
    supervisor.movieStartRecording(str(output.with_suffix(".mp4")), 1280, 720, 0, 85, 1, False)
    result = evaluate(env, count, seed, recorder=recorder)
    supervisor.movieStopRecording()
    deadline = time.monotonic() + 60
    while not supervisor.movieIsReady() and time.monotonic() < deadline:
        if supervisor.step(20) == -1:
            raise RuntimeError("Webots closed during movie encoding")
    if not supervisor.movieIsReady() or supervisor.movieFailed():
        raise RuntimeError("Native decision movie encoding failed")
    result.update(
        source="actual Webots movie recorder",
        scripted_target_selection=False,
        scripted_role_handoff=False,
        opponent_policy_fallback=False,
        outcome_display_seconds=0.6,
        post_terminal_control="referee HALT; excluded from scoring",
        possession_sample_seconds=0.005,
    )
    return result
