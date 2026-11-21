"""Native 3v3 blocked-outlet self-play controller."""

import json
import os
from pathlib import Path
import time
import traceback
from controller import Supervisor
import numpy as np
import torch

from roboturk.blocked_audit import opening_result
from roboturk.blocked_outlet import BlockedGame
from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.recovery_camera import movie_camera
from roboturk.strategy_pass import LearnedPass
from roboturk.three_match import CFG


def read(path):
    return json.loads(path.read_text())


def run(sim, folder):
    cfg = read(folder / "launch.json")
    root = Path(cfg["root"])
    torch.set_num_threads(1)
    motor = LearnedPass(root / "policies/strategy_self_play_v1/preparation/pass-motor-v5/policy.pt")
    prior = read(root / "policies/contest_outlet_v2/policy.json")
    selectors = read(root / "policies/recipient_3v3_v1/selectors.json")
    sides = ("blue", "yellow")
    jaw = {t: prior["jaw"] for t in sides}
    recovery = {
        t: dict(
            motor=np.array(prior["motor"]) / prior["temperature"], outlet=np.array(prior["outlet"])
        )
        for t in sides
    }
    weights = {t: np.array(selectors[str(901 + i)]) for i, t in enumerate(sides)}
    backend = ContinuousWebotsBackend(sim, CFG)
    render = movie_camera(sim, folder) if cfg["record"] else None
    case = os.environ.get("RECOVERY_CASE")
    for i in [int(case)] if case is not None else range(cfg["cases"]):
        seed, blue, yellow = cfg["schedule"][i]
        teams = dict(blue=cfg["models"][blue], yellow=cfg["models"][yellow])
        game = BlockedGame(
            backend,
            motor,
            weights,
            seed,
            cfg["seconds"],
            set(),
            teams=teams,
            strategies={t: True for t in sides},
            fixture=False,
            early_stop=cfg["study"],
            jaw_policies=jaw,
            policies=recovery,
            exploring=sides,
            trace=True,
            render=render,
        )
        result = game.run()
        states = np.asarray(game.samples)
        result.update(options=game.options, blue_model=blue, yellow_model=yellow)
        result["opening"] = opening_result(states, result, cfg["protocol"])
        if not cfg["study"]:
            np.savez_compressed(folder / f"trace-{i:03}.npz", states=states)
        (folder / f"game-{i:03}.json").write_text(json.dumps(result, separators=(",", ":")) + "\n")
        print(i, seed, result["opening"]["outcome"], flush=True)
    if render:
        sim.movieStopRecording()
        timeout = time.monotonic() + 120
        while not sim.movieIsReady() and time.monotonic() < timeout:
            sim.step(40)
        if not sim.movieIsReady() or sim.movieFailed():
            raise RuntimeError("Movie encoding failed")


sim = Supervisor()
folder = Path(os.environ["ATTACK_RUN"])
try:
    run(sim, folder)
    sim.simulationQuit(0)
except Exception:
    (folder / "error.txt").write_text(traceback.format_exc())
    sim.simulationQuit(1)
