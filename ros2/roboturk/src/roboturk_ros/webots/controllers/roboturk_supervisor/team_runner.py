import hashlib
import json
import os
from pathlib import Path
from roboturk.behavior import CheckpointPolicy
from roboturk.model import Skill
from roboturk.team import PassReceiveEnv, TeamCheckpoint, TransferTeacher, evaluate_team


def run(supervisor, backend, cfg, mode):
    env = PassReceiveEnv(cfg, backend)
    output = Path(os.environ.get("ROBOTURK_OUTPUT", "runs/team/result"))
    output.parent.mkdir(parents=True, exist_ok=True)
    if mode == "team-upgrade":
        from roboturk.team_upgrade import upgrade_team

        upgrade_team(
            env,
            os.environ["ROBOTURK_CHECKPOINT"],
            output,
            int(os.environ.get("ROBOTURK_STEPS", "65536")),
            int(os.environ.get("ROBOTURK_SEED", "57")),
        )
    elif mode == "team-train":
        from roboturk.team_training import train_team

        train_team(
            env,
            output,
            int(os.environ.get("ROBOTURK_STEPS", "65536")),
            int(os.environ.get("ROBOTURK_SEED", "47")),
        )
    else:
        checkpoint = os.environ.get("ROBOTURK_CHECKPOINT")
        policy = TeamCheckpoint(checkpoint, cfg) if checkpoint else TransferTeacher()
        shooter = (
            CheckpointPolicy(os.environ["ROBOTURK_SHOOT_CHECKPOINT"], Skill.SHOOT, cfg)
            if os.environ.get("ROBOTURK_SHOOT_CHECKPOINT")
            else None
        )
        episodes, seed = (
            int(os.environ.get("ROBOTURK_EPISODES", "200")),
            int(os.environ.get("ROBOTURK_SEED", "450000")),
        )
        if mode == "team-demo":
            if not checkpoint or shooter is None:
                raise ValueError("Demo requires both learned transfer and scoring checkpoints")
            from team_demo import record

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
