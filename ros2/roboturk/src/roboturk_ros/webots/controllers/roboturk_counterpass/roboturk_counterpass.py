"""Native continuous 3v3 self-play with ordinary restarts."""

from dataclasses import asdict
import json
import os
from pathlib import Path
import time
import traceback
from controller import Supervisor
import numpy as np
import torch

from roboturk.open_play_audit import recoveries
from roboturk.counterpass import DistributionGame
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
        game = DistributionGame(
            backend,
            motor,
            weights,
            seed,
            cfg.get("durations", [cfg["seconds"]] * cfg["cases"])[i],
            set(),
            teams=teams,
            decision_start=cfg.get("decision_starts", [None] * cfg["cases"])[i],
            press_from=cfg.get("press_from", [0] * cfg["cases"])[i],
            takeaway_start=cfg.get("starts", [None] * cfg["cases"])[i],
            press=cfg["press"],
            strategies={t: True for t in sides},
            fixture=False,
            early_stop=False,
            jaw_policies={t: teams[t].get("recovery_jaw", prior["jaw"]) for t in sides},
            policies=recovery,
            exploring=sides,
            trace=True,
            render=render,
        )
        initial_world = asdict(game.world)
        result = game.run()
        result["initial_world"] = initial_world
        states = np.asarray(game.samples)
        result.update(options=game.options, blue_model=blue, yellow_model=yellow)
        result["takeaway_actions"] = game.takeaway_actions
        result["recoveries"] = recoveries(states, result)
        np.savez_compressed(folder / f"trace-{i:03}.npz", states=states)
        (folder / f"game-{i:03}.json").write_text(json.dumps(result, separators=(",", ":")) + "\n")
        print(i, seed, len(result["recoveries"]), flush=True)
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
