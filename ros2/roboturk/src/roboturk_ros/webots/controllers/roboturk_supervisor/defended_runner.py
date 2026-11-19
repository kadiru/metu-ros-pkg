import hashlib
import json
import os
from pathlib import Path
from roboturk.behavior import CheckpointPolicy
from roboturk.model import Skill
from roboturk.team import TeamCheckpoint, evaluate_team
from roboturk.defended import (
    DefendedEnv,
    DefendedCheckpoint,
    DefendedTeacher,
    DEFENDED_CONTRACT,
    DEFENDED_CONTRACT_V1,
    DEFENDED_CONTRACT_V2,
)


def run(supervisor, backend, cfg, mode):
    env = DefendedEnv(
        cfg,
        backend,
        contract=DEFENDED_CONTRACT_V2
        if mode in ("defended-train", "defended-upgrade")
        else DEFENDED_CONTRACT,
    )
    output = Path(os.environ.get("ROBOTURK_OUTPUT", "runs/team/result"))
    output.parent.mkdir(parents=True, exist_ok=True)
    if mode == "defended-tactical-train":
        from roboturk.defended_tactical_training import train_tactical

        train_tactical(
            env,
            output,
            int(os.environ.get("ROBOTURK_STEPS", "65536")),
            int(os.environ.get("ROBOTURK_SEED", "87")),
        )
    elif mode == "defended-train":
        from roboturk.defended_training import train_defended

        train_defended(
            env,
            output,
            int(os.environ.get("ROBOTURK_STEPS", "65536")),
            int(os.environ.get("ROBOTURK_SEED", "67")),
        )
    elif mode == "defended-upgrade":
        from roboturk.defended_upgrade import upgrade_defended

        upgrade_defended(
            env,
            os.environ["ROBOTURK_CHECKPOINT"],
            output,
            int(os.environ.get("ROBOTURK_STEPS", "65536")),
            int(os.environ.get("ROBOTURK_SEED", "77")),
        )
    else:
        checkpoint = os.environ.get("ROBOTURK_CHECKPOINT")
        policy = (
            (
                DefendedCheckpoint(checkpoint, cfg)
                if json.loads(Path(checkpoint).with_suffix(".json").read_text())["contract"]
                in (DEFENDED_CONTRACT, DEFENDED_CONTRACT_V2, DEFENDED_CONTRACT_V1)
                else TeamCheckpoint(checkpoint, cfg)
            )
            if checkpoint
            else DefendedTeacher()
        )
        shooter = (
            CheckpointPolicy(os.environ["ROBOTURK_SHOOT_CHECKPOINT"], Skill.SHOOT, cfg)
            if os.environ.get("ROBOTURK_SHOOT_CHECKPOINT")
            else None
        )
        episodes, seed = (
            int(os.environ.get("ROBOTURK_EPISODES", "200")),
            int(os.environ.get("ROBOTURK_SEED", "650000")),
        )
        if mode == "defended-demo":
            if not checkpoint or shooter is None:
                raise ValueError("Demo requires both learned transfer and scoring checkpoints")
            from defended_demo import record

            result = record(supervisor, env, policy, shooter, output, episodes, seed)
        else:
            result = evaluate_team(env, policy, episodes, seed, shooter)
        result["policy"] = checkpoint or "analytic_demonstration_teacher"
        result["shooter"] = os.environ.get("ROBOTURK_SHOOT_CHECKPOINT")
        for label, path in [("checkpoint", checkpoint), ("shooting_checkpoint", result["shooter"])]:
            if path:
                result[label + "_sha256"] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({k: v for k, v in result.items() if k != "episodes"}), flush=True)
    env.close()
    supervisor.simulationQuit(0)
