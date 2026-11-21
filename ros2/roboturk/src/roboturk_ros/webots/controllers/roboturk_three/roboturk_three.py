"""Native receiver-choice self-play, evaluation and movie recording."""

from collections import Counter
import json
import math
import os
from pathlib import Path
import time
import traceback

from controller import Supervisor
import numpy as np
import torch
from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.strategy_pass import LearnedPass
from roboturk.three_match import Game, CFG, DT
from roboturk.three_recipient import train_update


def save(path, data):
    path.write_text(json.dumps(data, indent=2) + "\n")


def run(supervisor, folder):
    cfg = json.loads((folder / "launch.json").read_text())
    mode, seed = cfg["mode"], cfg["seed"]
    release = Path(cfg["root"]) / "policies"
    torch.set_num_threads(1)
    motor = LearnedPass(release / "strategy_self_play_v1/preparation/pass-motor-v5/policy.pt")
    backend = ContinuousWebotsBackend(supervisor, CFG)
    rng = np.random.default_rng(seed)
    initial = rng.normal(0, 0.02, 6)
    history = [initial.copy()]
    weights = initial.copy()
    models = json.loads(Path(cfg["models"]).read_text()) if cfg["models"] else {}
    results, updates = [], []
    demo = mode == "demo"
    if demo:
        ring_points = ", ".join(
            f"{0.075 * math.cos(a * math.tau / 32)} {0.075 * math.sin(a * math.tau / 32)} 0"
            for a in range(32)
        )
        indices = ", ".join(str(n) for n in [*range(32), 0, -1])
        children = supervisor.getRoot().getField("children")
        children.importMFNodeFromString(
            -1,
            "DEF THREE_BALL_RING Pose { children [ Shape { "
            "appearance Appearance { material Material { emissiveColor 1 .8 .1 } } "
            f"geometry IndexedLineSet {{ coord Coordinate {{ point [ {ring_points} ] }} "
            f"coordIndex [ {indices} ] }} }} ] }}",
        )
        ring = supervisor.getFromDef("THREE_BALL_RING").getField("translation")
        camera = supervisor.getFromDef("CAMERA")
        camera.getField("position").setSFVec3f([0, 0, 16.5])
        camera.getField("orientation").setSFRotation(
            [-0.577350269, 0.577350269, 0.577350269, 2.094395102]
        )
        supervisor.movieStartRecording(str(folder / "match.mp4"), 1280, 720, 0, 85, 1, False)
    for i in range(cfg["matches"]):
        learner = "yellow" if i % 2 else "blue"
        opp = "yellow" if learner == "blue" else "blue"
        if mode == "train":
            opponent = int(rng.integers(len(history)))
            policies = {learner: weights, opp: history[opponent]}
            sample = {learner, opp}
        else:
            panel = i // 8 if mode == "qualify" else 2
            blue, yellow = [(901, 901), (902, 902), (901, 902), (902, 901)][panel]
            policies = {
                "blue": np.array(models[str(blue)]),
                "yellow": np.array(models[str(yellow)]),
            }
            sample = set()
            if mode == "control":
                challenger = "blue" if i % 2 == 0 else "yellow"
                policies[challenger] = np.zeros(6)
                sample.add(challenger)

        def render(game):
            ring.setSFVec3f([game.world.ball.x, game.world.ball.y, 0.045])

            def label(index, line, y, color=0xFFFFFF):
                supervisor.setLabel(index, line, 0.02, y, 0.052, color, 0, "Arial")

            label(
                0,
                f"RoboTurk   3 v 3     |     Match {i + 1}     |     {game.tick * DT:05.1f} s",
                0.02,
            )
            label(1, f"BLUE  {game.score['blue']}   :   {game.score['yellow']}  YELLOW", 0.09)
            completed = sum(p.get("result") == "complete" for p in game.passes)
            label(2, f"Intended receives   {completed} / {len(game.passes)} passes", 0.16)
            last = game.passes[-1] if game.passes else None
            if game.choice is not None and game.owner is not None:
                label(
                    3,
                    f"{game.owner.team.upper()} {game.owner.id}  selects  {game.choice.receiver}",
                    0.84,
                    0xFFFF80,
                )
            elif not last:
                label(3, "First possession", 0.84)
            else:
                label(
                    3,
                    f"{last['team'].upper()} {last['passer']}  to  {last['receiver']}     "
                    + last.get("result", "in flight"),
                    0.84,
                    0xFFFF80,
                )
            for k in backend.nodes:
                r = game.world.robot(k)
                supervisor.setLabel(
                    4 + k.id + (3 if k.team == "yellow" else 0),
                    str(k.id),
                    0.502 + r.x * 0.073,
                    0.486 - r.y * 0.130,
                    0.032,
                    0xFFFFFF,
                    0,
                    "Arial",
                )

        game = Game(
            backend,
            motor,
            policies,
            cfg["first"] + i,
            cfg["seconds"],
            sample,
            render if demo else None,
            mode in ("demo", "qualify"),
        )
        result = game.run()
        if game.trace:
            np.savez_compressed(
                folder / f"trace-{i:02}.npz", states=np.asarray(game.samples, dtype=np.float32)
            )
        if mode == "train":
            rows = [p for p in game.passes if p["team"] == learner]
            before = weights.copy()
            weights = train_update(weights, rows, rate=2.0)
            updates.append(
                dict(
                    learner=learner,
                    opponent_checkpoint=opponent,
                    choices=len(rows),
                    before=before.tolist(),
                    after=weights.tolist(),
                )
            )
            if (i + 1) % 4 == 0:
                history.append(weights.copy())
        result["outcomes"] = dict(Counter(p["result"] for p in game.passes))
        results.append(result)
        print(i + 1, result["outcomes"], result["score"], flush=True)
        save(folder / f"game-{i:02}.json", result)
    if demo:
        supervisor.movieStopRecording()
        deadline = time.monotonic() + 120
        while not supervisor.movieIsReady() and time.monotonic() < deadline:
            supervisor.step(40)
        if not supervisor.movieIsReady() or supervisor.movieFailed():
            raise RuntimeError("Native movie encoding failed")
    if mode == "train":
        save(
            folder / "training.json",
            dict(
                seed=seed,
                initial=initial.tolist(),
                final=weights.tolist(),
                history=[w.tolist() for w in history],
                updates=updates,
            ),
        )
    save(
        folder / "results.json",
        [
            dict(seed=r["seed"], score=r["score"], outcomes=r["outcomes"], stats=r["stats"])
            for r in results
        ],
    )


supervisor = Supervisor()
folder = Path(os.environ["THREE_RUN"])
try:
    run(supervisor, folder)
    supervisor.simulationQuit(0)
except Exception:
    (folder / "error.txt").write_text(traceback.format_exc())
    supervisor.simulationQuit(1)
