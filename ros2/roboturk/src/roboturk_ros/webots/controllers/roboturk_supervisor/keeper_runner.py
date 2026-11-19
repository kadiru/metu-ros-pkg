import hashlib
import json
import os
from pathlib import Path
from roboturk.keeper import (
    KeeperBackend,
    KeeperFinishEnv,
    KeeperPlayEnv,
    FinishTeacher,
    FinishCheckpoint,
)
from roboturk.defended import DefendedCheckpoint
from roboturk.team import evaluate_team
from roboturk.shoot import ShootEnv
from roboturk.behavior import CheckpointPolicy
from roboturk.model import Skill


def run(supervisor, backend, cfg, mode):
    output = Path(os.environ["ROBOTURK_OUTPUT"])
    output.parent.mkdir(parents=True, exist_ok=True)
    episodes, seed = int(os.environ["ROBOTURK_EPISODES"]), int(os.environ["ROBOTURK_SEED"])
    checkpoint = os.environ.get("ROBOTURK_CHECKPOINT")
    if mode in ("keeper-train", "keeper-finish-evaluate"):
        env = KeeperFinishEnv(cfg, KeeperBackend(backend, cfg))
        from roboturk.micro_training import evaluate, train

        if mode == "keeper-train":
            train(env, FinishTeacher(), output, int(os.environ["ROBOTURK_STEPS"]), seed, "keeper")
            env.close()
            supervisor.simulationQuit(0)
            return
        result = evaluate(
            env,
            FinishCheckpoint(checkpoint, cfg) if checkpoint else FinishTeacher(),
            episodes,
            seed,
        )
    else:
        env = KeeperPlayEnv(cfg, backend)
        transfer = DefendedCheckpoint(checkpoint, cfg)
        shooter_path = os.environ.get("ROBOTURK_SHOOT_CHECKPOINT")
        if mode == "keeper-baseline":
            env.scoring_env_type = ShootEnv
            shooter = CheckpointPolicy(shooter_path, Skill.SHOOT, cfg)
        else:
            shooter = FinishCheckpoint(shooter_path, cfg) if shooter_path else FinishTeacher()
        if mode == "keeper-demo":
            if not shooter_path:
                raise ValueError("Keeper demo requires a learned finishing checkpoint")
            from keeper_demo import record

            result = record(supervisor, env, transfer, shooter, output, episodes, seed)
        else:
            result = evaluate_team(env, transfer, episodes, seed, shooter)
        result["transfer_checkpoint"] = checkpoint
        result["finish_checkpoint"] = shooter_path or "analytic_teacher"
    result["backend"] = env.backend.name
    for label, path in [
        ("checkpoint", checkpoint),
        ("shooting_checkpoint", os.environ.get("ROBOTURK_SHOOT_CHECKPOINT")),
    ]:
        if path:
            result[label + "_sha256"] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
    output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "episodes"}), flush=True)
    env.close()
    supervisor.simulationQuit(0)
