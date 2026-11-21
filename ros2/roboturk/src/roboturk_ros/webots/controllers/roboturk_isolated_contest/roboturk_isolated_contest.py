"""Evaluate exactly one frozen contest match in a fresh Webots process."""

import json
import os
from pathlib import Path
import traceback

from controller import Supervisor
import numpy as np
import torch

from roboturk.contest_metrics import measures
from roboturk.contest_outlet import ContestGame
from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.strategy_pass import LearnedPass
from roboturk.three_match import CFG


def run(sim, folder, index):
    launch = json.loads((folder / "launch.json").read_text())
    root = Path(launch["root"])
    torch.set_num_threads(1)
    motor = LearnedPass(root / "policies/strategy_self_play_v1/preparation/pass-motor-v5/policy.pt")
    both = json.loads((root / "policies/recipient_3v3_v1/selectors.json").read_text())
    selectors = dict(blue=np.array(both["901"]), yellow=np.array(both["902"]))
    policy = {k: np.array(v) for k, v in json.loads(Path(launch["model"]).read_text()).items()}
    team = "yellow" if index % 2 else "blue"
    mode = launch["mode"]
    sides = {
        t: policy if mode == "trained" or (mode == "head_to_head" and t == team) else None
        for t in selectors
    }
    game = ContestGame(
        ContinuousWebotsBackend(sim, CFG),
        motor,
        selectors,
        launch["first"] + index,
        launch["seconds"],
        set(),
        trace=True,
        policies=sides,
    )
    result = game.run()
    states = np.asarray(game.samples, dtype=np.float64)
    result.update(
        contest_metrics=measures(states, result),
        learned_actions=game.decisions,
        challenger=team if mode == "head_to_head" else None,
    )
    np.savez_compressed(folder / f"trace-{index:03}.npz", states=states)
    (folder / f"game-{index:03}.json").write_text(json.dumps(result, indent=2) + "\n")


sim = Supervisor()
folder = Path(os.environ["CONTEST_RUN"])
index = int(os.environ["CONTEST_INDEX"])
try:
    run(sim, folder, index)
    sim.simulationQuit(0)
except Exception:
    (folder / f"error-{index:03}.txt").write_text(traceback.format_exc())
    sim.simulationQuit(1)
