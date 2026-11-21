"""Native evaluation and recording for the return-trained contest policy."""

import json
import math
import os
from pathlib import Path
import time
import traceback

from controller import Supervisor
import numpy as np
import torch
from roboturk.recovery_conversion import conversions
from roboturk.recovery_attack import AttackGame
from roboturk.attack_returns import AttackLearner
from roboturk.attack_opening import fit as fit_opening, training_example

from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.strategy_pass import LearnedPass
from roboturk.three_match import CFG


def save(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n")


def movie_camera(sim, folder):
    camera = sim.getFromDef("CAMERA")
    camera.getField("position").setSFVec3f([0, 0, 16.5])
    camera.getField("orientation").setSFRotation(
        [-0.577350269, 0.577350269, 0.577350269, 2.094395102]
    )
    pts = ", ".join(
        f"{0.08 * math.cos(i * math.tau / 32)} {0.08 * math.sin(i * math.tau / 32)} 0"
        for i in range(32)
    )
    indices = ", ".join(str(j) for j in [*range(32), 0, -1])
    sim.getRoot().getField("children").importMFNodeFromString(
        -1,
        "DEF BALL_RING Pose { children [ Shape { appearance Appearance { material Material { emissiveColor 1 .8 .1 } } "
        f"geometry IndexedLineSet {{ coord Coordinate {{ point [ {pts} ] }} coordIndex [ {indices} ] }} }} ] }}",
    )
    ring = sim.getFromDef("BALL_RING").getField("translation")
    sim.movieStartRecording(str(folder / "match.mp4"), 1280, 720, 0, 85, 1, False)

    def draw(g):
        ring.setSFVec3f([g.world.ball.x, g.world.ball.y, 0.05])
        sim.setLabel(
            0,
            f"RoboTurk    |    3v3 open play    |    {g.tick * 0.04:05.1f} s",
            0.02,
            0.02,
            0.056,
            0xFFFFFF,
            0,
            "Arial",
        )
        sim.setLabel(
            1,
            f"BLUE   {g.score['blue']} : {g.score['yellow']}   YELLOW",
            0.02,
            0.10,
            0.055,
            0xFFFFFF,
            0,
            "Arial",
        )
        if g.dual_ticks >= 5:
            label = "CONTESTED BALL"
        elif g.flight:
            label = (
                f"{g.flight['team'].upper()} {g.flight['passer']} passes to {g.flight['receiver']}"
            )
        elif g.owner:
            label = f"{g.owner.team.upper()} {g.owner.id} in possession"
        elif g.passes:
            label = g.passes[-1].get("result", "").upper()
        else:
            label = "Ball in play"
        sim.setLabel(2, label, 0.02, 0.83, 0.065, 0xFFDF80, 0, "Arial")
        outlets = [
            f"{team.upper()} {role['mate'].id} opens for {role['actor'].id}"
            for team, role in g.roles.items()
        ]
        if g.active:
            team = g.active[0].upper()
            action = g.option["action"].upper() if g.option else "BALL IN PLAY"
            outlets = [f"{team} RECOVERY   |   {action}"]
        if g.events and g.events[-1]["event"] == "goal" and g.tick - g.events[-1]["tick"] < 28:
            label = "GOAL"
            sim.setLabel(2, label, 0.02, 0.83, 0.065, 0xFFDF80, 0, "Arial")
        sim.setLabel(10, "     ".join(outlets), 0.02, 0.91, 0.045, 0x83EEDD, 0, "Arial")
        for r in g.world.robots:
            sim.setLabel(
                3 + (3 if r.key.team == "yellow" else 0) + r.key.id,
                str(r.key.id),
                0.5 + r.x * 0.073,
                0.483 - r.y * 0.130,
                0.038,
                0xFFFFFF,
                0,
                "Arial",
            )

    return draw


def run(sim, folder):
    cfg = json.loads((folder / "launch.json").read_text())
    root = Path(cfg["root"])
    torch.set_num_threads(1)
    motor = LearnedPass(root / "policies/strategy_self_play_v1/preparation/pass-motor-v5/policy.pt")
    base = json.loads((root / "policies/contest_outlet_v2/policy.json").read_text())
    jaw = {t: base["jaw"] for t in ("blue", "yellow")}
    recover = {
        t: dict(
            motor=np.array(base["motor"]) / base["temperature"], outlet=np.array(base["outlet"])
        )
        for t in jaw
    }
    sel = json.loads((root / "policies/recipient_3v3_v1/selectors.json").read_text())
    weights = dict(blue=np.array(sel["901"]), yellow=np.array(sel["902"]))
    backend = ContinuousWebotsBackend(sim, CFG)
    mode = cfg["mode"]
    train = mode == "train"
    model = json.loads(Path(cfg["model"]).read_text()) if not train or cfg.get("refine") else {}
    refined = cfg.get("refine", False) if train else model.get("refined", False)
    fit = AttackLearner(model["q"] if train and refined else None)
    fixed = None if train else np.array(model["q"])
    history = [fit.policy()]
    examples = []
    branches = mode == "branches"
    render = movie_camera(sim, folder) if mode in ("demo", "showcase") else None
    idx = os.environ.get("RECOVERY_CASE")
    for i in [int(idx)] if idx is not None else range(cfg["cases"]):
        team = "blue" if i % 2 == 0 else "yellow"
        other = "yellow" if i % 2 == 0 else "blue"
        strategies = {t: fixed for t in jaw}
        opponent = None
        if train:
            strategies[team] = fit.policy()
            opponent = int(np.random.default_rng(cfg["first"] + i).integers(len(history)))
            strategies[other] = history[opponent]
        elif mode == "released":
            strategies = dict(blue=None, yellow=None)
        elif mode == "mixed":
            strategies[other] = None
        g = AttackGame(
            backend,
            motor,
            weights,
            cfg["first"] + (i // 3 if branches else i),
            cfg["seconds"],
            set(),
            strategies=strategies,
            refined=refined,
            early_stop=(train and refined) or branches,
            opening=model.get("opening"),
            opening_action=i % 3 if branches else None,
            precision=branches or "opening" in model,
            lead=model.get("lead", False),
            temperature=model.get("temperature", 0.55),
            fixture=mode != "ordinary" and not (mode == "demo" and i == 2),
            epsilon=max(0.22, 0.85 - 0.6 * i / max(1, cfg["cases"] - 1)),
            learner=team if train else None,
            jaw_policies=jaw,
            policies=recover,
            exploring=tuple(jaw),
            trace=True,
            render=render,
        )
        result = g.run()
        trace = np.asarray(g.samples)
        result.update(
            options=g.options,
            learner=team if train else None,
            challenger=team if mode == "mixed" else None,
            opponent=opponent,
        )
        if refined:
            for j, o in enumerate(g.options):
                end = next(
                    (v["tick"] for v in g.options[j + 1 :] if v["team"] == o["team"]), o["until"]
                )
                reset = next(
                    (
                        v["tick"] + 1
                        for v in g.events
                        if v["event"] not in ("start", "kick") and v["tick"] >= o["tick"]
                    ),
                    o["until"],
                )
                o["end_tick"] = min(o["until"], end, reset)
        result["conversions"] = conversions(trace, result)
        if branches:
            sample = training_example(result)
            if sample is not None:
                examples.append(sample)
        elif train:
            fit.update(result)
            if (i + 1) % 12 == 0:
                history.append(fit.policy())
                save(
                    folder / f"checkpoint-{i + 1:03}.json",
                    dict(q=history[-1].tolist(), refined=refined, temperature=0.55),
                )
        else:
            np.savez_compressed(folder / f"trace-{i:03}.npz", states=trace)
        save(folder / f"game-{i:03}.json", result)
        print(i, len(g.options), sum("goal" in c for c in result["conversions"]), flush=True)
    if branches:
        save(folder / "policy.json", dict(**model, opening=fit_opening(examples)))
    if train:
        save(
            folder / "policy.json",
            dict(
                q=fit.policy().tolist(),
                option_updates=fit.counts.tolist(),
                refined=refined,
                temperature=0.55,
            ),
        )
    if render:
        sim.movieStopRecording()
        deadline = time.monotonic() + 120
        while not sim.movieIsReady() and time.monotonic() < deadline:
            sim.step(40)
        if not sim.movieIsReady() or sim.movieFailed():
            raise RuntimeError("Webots recording failed")


sim = Supervisor()
folder = Path(os.environ["ATTACK_RUN"])
try:
    run(sim, folder)
    sim.simulationQuit(0)
except Exception:
    (folder / "error.txt").write_text(traceback.format_exc())
    sim.simulationQuit(1)
