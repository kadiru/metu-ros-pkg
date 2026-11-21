import hashlib
import json
import os
from pathlib import Path
from roboturk.behavior import CheckpointPolicy
from roboturk.shoot import ShootEnv, ShootTeacher, evaluate_shoot
from roboturk.model import Skill


def run(supervisor, backend, cfg, mode):
    env = ShootEnv(cfg, backend)
    output = Path(os.environ.get("ROBOTURK_OUTPUT", "runs/shoot/result"))
    output.parent.mkdir(parents=True, exist_ok=True)
    if mode == "shoot-upgrade":
        from roboturk.shoot_upgrade import upgrade_shoot

        checkpoint = os.environ.get("ROBOTURK_CHECKPOINT")
        if not checkpoint:
            raise ValueError("shoot-upgrade requires the v1 warm-start checkpoint and dataset")
        upgrade_shoot(
            env,
            checkpoint,
            output,
            int(os.environ.get("ROBOTURK_STEPS", "65536")),
            int(os.environ.get("ROBOTURK_SEED", "37")),
        )
    elif mode == "shoot-train":
        from roboturk.shoot_training import train_shoot

        train_shoot(
            env,
            output,
            int(os.environ.get("ROBOTURK_STEPS", "65536")),
            int(os.environ.get("ROBOTURK_SEED", "27")),
        )
    else:
        checkpoint = os.environ.get("ROBOTURK_CHECKPOINT")
        policy = CheckpointPolicy(checkpoint, Skill.SHOOT, cfg) if checkpoint else ShootTeacher()
        episodes, seed = (
            int(os.environ.get("ROBOTURK_EPISODES", "200")),
            int(os.environ.get("ROBOTURK_SEED", "250000")),
        )
        if mode == "shoot-demo":
            if not checkpoint:
                raise ValueError("shoot-demo requires a learned checkpoint")
            from shoot_demo import record

            result = record(supervisor, env, policy, output, episodes, seed)
        else:
            result = evaluate_shoot(env, policy, episodes, seed)
        result["policy"] = checkpoint or "analytic_demonstration_teacher"
        if checkpoint:
            result["checkpoint_sha256"] = hashlib.sha256(Path(checkpoint).read_bytes()).hexdigest()
        output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({k: v for k, v in result.items() if k != "episodes"}), flush=True)
    env.close()
    supervisor.simulationQuit(0)
