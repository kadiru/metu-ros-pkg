"""Native self-play for pressure-aware recovery attacks with the frozen v1 strategy."""

import json
import os
from pathlib import Path
import time
import traceback

from controller import Supervisor
import numpy as np
import torch
from roboturk.attack_opening import training_example
from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.recovery_camera import movie_camera
from roboturk.recovery_conversion import conversions
from roboturk.recovery_shield import ShieldGame
from roboturk.strategy_pass import LearnedPass
from roboturk.three_match import CFG


def read(path):
    return json.loads(path.read_text())


def save(path, obj):
    path.write_text(json.dumps(obj, separators=(",", ":")) + "\n")


def run(sim, folder):
    cfg = read(folder / "launch.json")
    root = Path(cfg["root"])
    torch.set_num_threads(1)
    motor = LearnedPass(root / "policies/strategy_self_play_v1/preparation/pass-motor-v5/policy.pt")
    base = read(root / "policies/contest_outlet_v2/policy.json")
    sides = ("blue", "yellow")
    jaw = {t: base["jaw"] for t in sides}
    recovery = {
        t: dict(
            motor=np.array(base["motor"]) / base["temperature"], outlet=np.array(base["outlet"])
        )
        for t in sides
    }
    sel = read(root / "policies/recipient_3v3_v1/selectors.json")
    weights = dict(blue=np.array(sel["901"]), yellow=np.array(sel["902"]))
    backend = ContinuousWebotsBackend(sim, CFG)
    mode = cfg["mode"]
    model = read(Path(cfg["model"]))
    render = movie_camera(sim, folder) if cfg.get("record") else None
    index = os.environ.get("RECOVERY_CASE")
    for i in [int(index)] if index is not None else range(cfg["cases"]):
        study = mode in ("motors", "branches")
        variants = cfg.get("controls", [model.get("control")])
        variant = (i + i // len(variants)) % len(variants) if mode == "motors" else 0
        repeats = len(variants) if mode == "motors" else 3 if mode == "branches" else 1
        seed = cfg["first"] + i // repeats
        controls = {t: variants[variant] for t in sides}
        q = {t: np.asarray(model["q"]) for t in sides}
        challenger = sides[i % 2] if mode == "mixed" else None
        for t in sides:
            if mode == "released" or (mode == "mixed" and t != challenger):
                q[t] = None
                controls[t] = None
        g = ShieldGame(
            backend,
            motor,
            weights,
            seed,
            cfg["seconds"],
            set(),
            strategies=q,
            refined=True,
            early_stop=study,
            opening=model["opening"],
            opening_action=i % 3 if mode == "branches" else None,
            precision=True,
            lead=True,
            temperature=model["temperature"],
            fixture=mode != "ordinary" and not (mode == "demo" and i == 2),
            controls=controls,
            jaw_policies=jaw,
            policies=recovery,
            exploring=sides,
            trace=True,
            render=render,
        )
        result = g.run()
        states = np.asarray(g.samples)
        result.update(options=g.options, challenger=challenger, control=variant)
        result["conversions"] = conversions(states, result)
        if study:
            result["opening_example"] = training_example(result)
        else:
            np.savez_compressed(folder / f"trace-{i:03}.npz", states=states)
        save(folder / f"game-{i:03}.json", result)
        print(i, seed, flush=True)
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
