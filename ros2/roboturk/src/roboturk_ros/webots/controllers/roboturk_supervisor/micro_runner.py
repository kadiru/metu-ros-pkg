import json
import os
from pathlib import Path
from roboturk.defense_micro import DefenseMicroEnv, DefenseTeacher, DefenseCheckpoint
from roboturk.micro_training import evaluate, train


def run(supervisor, backend, cfg, mode):
    kind = os.environ.get("ROBOTURK_MICRO", "block")
    env = DefenseMicroEnv(cfg, backend, kind)
    output = Path(os.environ["ROBOTURK_OUTPUT"])
    output.parent.mkdir(parents=True, exist_ok=True)
    if mode == "micro-train":
        train(
            env,
            DefenseTeacher(),
            output,
            int(os.environ["ROBOTURK_STEPS"]),
            int(os.environ["ROBOTURK_SEED"]),
            kind,
        )
    else:
        path = os.environ.get("ROBOTURK_CHECKPOINT")
        actor = DefenseCheckpoint(path, env) if path else DefenseTeacher()
        result = evaluate(
            env, actor, int(os.environ["ROBOTURK_EPISODES"]), int(os.environ["ROBOTURK_SEED"])
        )
        result.update(backend=env.backend.name, policy=path or "analytic_teacher")
        output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({k: v for k, v in result.items() if k != "episodes"}), flush=True)
    env.close()
    supervisor.simulationQuit(0)
