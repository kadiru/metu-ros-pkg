"""Webots controller for the contested-possession and support policies."""

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
from roboturk.contest_outlet import ContestGame, apply_gradient, initial
from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.strategy_pass import LearnedPass
from roboturk.three_match import CFG


def save(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n")


def packed(policy):
    return {k: v.tolist() for k, v in policy.items()}


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
    mode = cfg["mode"]
    root = Path(cfg["root"])
    torch.set_num_threads(1)
    motor = LearnedPass(root / "policies/strategy_self_play_v1/preparation/pass-motor-v5/policy.pt")
    backend = ContinuousWebotsBackend(sim, CFG)
    weights = json.loads((root / "policies/recipient_3v3_v1/selectors.json").read_text())
    selectors = dict(blue=np.array(weights["901"]), yellow=np.array(weights["902"]))
    rng = np.random.default_rng(cfg["seed"])
    policy = (
        initial(rng)
        if mode == "train"
        else {k: np.array(v) for k, v in json.loads(Path(cfg["model"]).read_text()).items()}
    )
    history = [{k: v.copy() for k, v in policy.items()}]
    summary = []
    updates = []
    render = movie_camera(sim, folder) if mode == "demo" else None
    for i in range(cfg["matches"]):
        team = "blue" if i % 2 == 0 else "yellow"
        other = "yellow" if team == "blue" else "blue"
        sides = {"blue": policy, "yellow": policy}
        exploring = ()
        opponent = None
        if mode == "train":
            opponent = int(rng.integers(len(history))) if i % 3 else -1
            sides[other] = history[opponent] if opponent >= 0 else None
            exploring = (team, other)
        elif mode == "released":
            sides = dict(blue=None, yellow=None)
        elif mode == "head_to_head":
            sides[other] = None
        g = ContestGame(
            backend,
            motor,
            selectors,
            cfg["first"] + i,
            cfg["seconds"],
            set(),
            render=render,
            trace=True,
            policies=sides,
            exploring=exploring,
            duel=mode in ("train", "develop") or (mode == "demo" and i == 0),
        )
        result = g.run()
        states = np.array(g.samples, dtype=np.float64)
        analysis = measures(states, result)
        result["contest_metrics"] = analysis
        result["learned_actions"] = g.decisions
        result["challenger"] = team if mode in ("head_to_head", "train") else None
        if mode == "train":
            ev = analysis["contests"][0] if analysis["contests"] else {}
            own = ev.get("winner", -1) // 3 == (0 if team == "blue" else 1)
            # Per-episode credit ends at the first sustained recovery. Success
            # in a later unrelated possession cannot reward the initial duel.
            end = ev.get("recovery", len(states)) // 4
            decisions = [a for a in g.decisions if a["team"] == team and a["tick"] <= end]
            reward = (1 + 3 * ("received" in ev)) if own else -1
            outlet_reward = 0
            if own:
                targets = [a["mate"] for a in decisions if a["head"] == "outlet"]
                delivered = "receiver" in ev and ev["receiver"] % 3 in targets
                outlet_reward = 3 if delivered else -1
            apply_gradient(policy, [a for a in decisions if a["head"] == "motor"], reward, 0.65)
            apply_gradient(
                policy, [a for a in decisions if a["head"] == "outlet"], outlet_reward, 0.65
            )
            updates.append(
                dict(
                    seed=result["seed"],
                    learner=team,
                    opponent=opponent,
                    reward=reward,
                    outlet_reward=outlet_reward,
                    decisions=len(decisions),
                    after=packed(policy),
                )
            )
            if (i + 1) % 8 == 0:
                history.append({k: v.copy() for k, v in policy.items()})
        else:
            np.savez_compressed(folder / f"trace-{i:03}.npz", states=states)
        save(folder / f"game-{i:03}.json", result)
        summary.append(analysis)
        print(
            i + 1, analysis["contested_seconds"], analysis["chains"], analysis["late"], flush=True
        )
    if render:
        sim.movieStopRecording()
        deadline = time.monotonic() + 120
        while not sim.movieIsReady() and time.monotonic() < deadline:
            sim.step(40)
        if not sim.movieIsReady() or sim.movieFailed():
            raise RuntimeError("Native recording failed")
    if mode == "train":
        save(folder / "policy.json", packed(policy))
        save(folder / "updates.json", updates)
    save(folder / "results.json", summary)


sim = Supervisor()
folder = Path(os.environ["CONTEST_RUN"])
try:
    run(sim, folder)
    sim.simulationQuit(0)
except Exception:
    (folder / "error.txt").write_text(traceback.format_exc())
    sim.simulationQuit(1)
