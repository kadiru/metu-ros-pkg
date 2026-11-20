"""Single-owner native learned-decision experiments."""

import json
import os
from pathlib import Path
import torch
from roboturk.decision_play import DecisionEnv, evaluate
from roboturk.decision_training import DecisionTeacher, bootstrap, train, load
from roboturk.self_play import load_model, digest
from roboturk.self_play_training import write


def run(supervisor, backend, cfg, mode):
    torch.set_num_threads(1)
    output = Path(os.environ["ROBOTURK_OUTPUT"])
    root = Path(__file__).resolve().parents[5]
    count, seed = int(os.environ["ROBOTURK_EPISODES"]), int(os.environ["ROBOTURK_SEED"])
    sources = {
        r: str(root / "policies/self_play_v1/final" / r / "policy.zip")
        for r in ("transfer", "finish", "defender", "keeper")
    }
    if mode == "decision-bootstrap":
        bootstrap(backend, cfg, output, sources, seed)
    elif mode == "decision-train":
        train(
            backend,
            cfg,
            output,
            rounds=int(os.environ.get("ROBOTURK_ROUNDS", "3")),
            steps=int(os.environ["ROBOTURK_STEPS"]),
            seed=seed,
        )
    else:
        bundle_path = os.environ.get("ROBOTURK_CHECKPOINT")
        if bundle_path:
            bundle = json.loads(Path(bundle_path).read_text())
            models = load(bundle, cfg)
        else:
            models = {r: load_model(p, r, cfg) for r, p in sources.items()}
            bundle = {r: {"path": p, "sha256": digest(p)} for r, p in sources.items()}
        env = DecisionEnv(backend, cfg, models)
        if mode == "decision-probe":
            result = evaluate(env, count, seed, DecisionTeacher())
            result["manager_control"] = (
                "scripted training teacher; low-level motion and opponents neural"
            )
        elif mode == "decision-evaluate":
            result = evaluate(env, count, seed)
        elif mode == "decision-demo":
            from decision_demo import record

            result = record(supervisor, env, output, count, seed)
        else:
            raise ValueError("Unknown decision experiment")
        result["models"] = bundle
        write(output.with_suffix(".json"), result)
        print(json.dumps(result["summary"]), flush=True)
    backend.close()
    supervisor.simulationQuit(0)
