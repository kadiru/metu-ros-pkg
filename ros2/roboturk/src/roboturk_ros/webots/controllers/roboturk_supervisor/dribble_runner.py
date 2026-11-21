import hashlib
import json
import os
from pathlib import Path
from roboturk.behavior import CheckpointPolicy
from roboturk.dribble import DribbleEnv, DribbleTeacher, evaluate_dribble
from roboturk.model import Skill


def run(supervisor, backend, cfg, mode):
    env = DribbleEnv(cfg, backend)
    output = Path(os.environ.get("ROBOTURK_OUTPUT", "runs/dribble/result"))
    output.parent.mkdir(parents=True, exist_ok=True)
    if mode == "dribble-refine":
        from roboturk.dribble_refinement import refine_dribble

        checkpoint = os.environ.get("ROBOTURK_CHECKPOINT")
        if not checkpoint:
            raise ValueError(
                "dribble-refine requires a cloned checkpoint and its training observations"
            )
        refine_dribble(
            env,
            checkpoint,
            output,
            int(os.environ.get("ROBOTURK_STEPS", "65536")),
            int(os.environ.get("ROBOTURK_SEED", "18")),
        )
    elif mode == "dribble-train":
        from roboturk.dribble_training import train_dribble

        train_dribble(
            env,
            output,
            int(os.environ.get("ROBOTURK_STEPS", "131072")),
            int(os.environ.get("ROBOTURK_SEED", "17")),
        )
    else:
        checkpoint = os.environ.get("ROBOTURK_CHECKPOINT")
        policy = (
            CheckpointPolicy(checkpoint, Skill.DRIBBLE, cfg) if checkpoint else DribbleTeacher()
        )
        episodes, seed = (
            int(os.environ.get("ROBOTURK_EPISODES", "200")),
            int(os.environ.get("ROBOTURK_SEED", "150000")),
        )
        if mode == "dribble-demo":
            if not checkpoint:
                raise ValueError("dribble-demo requires a learned checkpoint")
            from dribble_demo import record

            result = record(supervisor, env, policy, output, episodes, seed)
        else:
            result = evaluate_dribble(env, policy, episodes, seed)
        result["policy"] = checkpoint or "analytic_demonstration_teacher"
        if checkpoint:
            result["checkpoint_sha256"] = hashlib.sha256(Path(checkpoint).read_bytes()).hexdigest()
        output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({k: v for k, v in result.items() if k != "episodes"}), flush=True)
    env.close()
    supervisor.simulationQuit(0)
