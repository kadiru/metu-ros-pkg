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
from roboturk.contest_metrics import measures
from roboturk.jaw_recovery import JawGame, ReturnLearner
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
    weights = json.loads((root / "policies/recipient_3v3_v1/selectors.json").read_text())
    selectors = dict(blue=np.array(weights["901"]), yellow=np.array(weights["902"]))
    model = json.loads(Path(cfg["model"]).read_text())
    policy = dict(
        motor=np.array(model["motor"]) / model["temperature"], outlet=np.array(model["outlet"])
    )
    backend = ContinuousWebotsBackend(sim, CFG)
    mode = cfg["mode"]
    render = movie_camera(sim, folder) if mode == "demo" else None
    idx = os.environ.get("CONTEST_INDEX")
    indexes = [int(idx)] if idx is not None else range(cfg["matches"])
    learner = ReturnLearner()
    history = [learner.policy()]
    training = mode == "train_jaw"
    poses = (
        json.loads((root / "policies/contest_outlet_v2/training-poses.json").read_text())
        if training
        else None
    )
    results = []
    for i in indexes:
        team = "yellow" if i % 2 else "blue"
        sides = {
            t: None if mode == "released" or (mode == "head_to_head" and t != team) else policy
            for t in selectors
        }
        jaws = {t: model.get("jaw") for t in selectors if sides[t] is not None}
        opponent = None
        if training:
            other = "yellow" if team == "blue" else "blue"
            jaws[team] = learner.policy()
            if i % 4 in (0, 3):
                sides[other] = None
                opponent = "released"
            elif i % 4 == 1:
                opponent = "previous"
            else:
                n = int(np.random.default_rng(cfg["first"] + i).integers(len(history)))
                jaws[other] = history[n]
                opponent = n
        game = JawGame(
            backend,
            motor,
            selectors,
            cfg["first"] + i,
            cfg["seconds"],
            set(),
            jaw_policies=jaws,
            poses=poses,
            training=team if training else False,
            render=render,
            trace=True,
            policies=sides,
            exploring=tuple(t for t, v in sides.items() if v is not None),
            duel=cfg.get("duel", False) or (mode == "demo" and i == 0),
        )
        result = game.run()
        states = np.asarray(game.samples, dtype=np.float64)
        metrics = measures(states, result)
        result.update(
            contest_metrics=metrics,
            learned_actions=game.decisions,
            challenger=team if mode == "head_to_head" or training else None,
        )
        if training:
            result["reward"] = learner.update(result)
            result["opponent"] = opponent
            if (i + 1) % 16 == 0:
                history.append(learner.policy())
        else:
            np.savez_compressed(folder / f"trace-{i:03}.npz", states=states)
        save(folder / f"game-{i:03}.json", result)
        results.append(metrics)
        print(i, metrics["chains"], flush=True)
    if training:
        save(folder / "policy.json", dict(**model, jaw=learner.policy()))
    if render:
        sim.movieStopRecording()
        deadline = time.monotonic() + 120
        while not sim.movieIsReady() and time.monotonic() < deadline:
            sim.step(40)
        if sim.movieFailed() or not sim.movieIsReady():
            raise RuntimeError("Webots movie encoding failed")
    if idx is None:
        save(folder / "results.json", results)


sim = Supervisor()
folder = Path(os.environ["CONTEST_RUN"])
try:
    run(sim, folder)
    sim.simulationQuit(0)
except Exception:
    (folder / "error.txt").write_text(traceback.format_exc())
    sim.simulationQuit(1)
