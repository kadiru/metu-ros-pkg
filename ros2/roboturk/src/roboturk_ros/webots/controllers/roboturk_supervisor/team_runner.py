import hashlib
import json
import os
from pathlib import Path
from roboturk.behavior import CheckpointPolicy
from roboturk.model import Skill
from roboturk.team import PassReceiveEnv, TeamCheckpoint, TransferTeacher, evaluate_team


def run(supervisor, backend, cfg, mode):
    if mode.startswith("team-closing-"):
        from roboturk.closing_press import ClosingPassEnv

        env = ClosingPassEnv(cfg, backend)
    else:
        env = PassReceiveEnv(cfg, backend)
    output = Path(os.environ.get("ROBOTURK_OUTPUT", "runs/team/result"))
    output.parent.mkdir(parents=True, exist_ok=True)
    if mode == "team-fit-receiver":
        from roboturk.receiver_fit import fit_receiver

        fit_receiver(
            env,
            os.environ["ROBOTURK_CHECKPOINT"],
            output,
            int(os.environ["ROBOTURK_EPISODES"]),
            int(os.environ["ROBOTURK_SEED"]),
        )
    elif mode == "team-upgrade":
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
        shooter = (
            CheckpointPolicy(os.environ["ROBOTURK_SHOOT_CHECKPOINT"], Skill.SHOOT, cfg)
            if os.environ.get("ROBOTURK_SHOOT_CHECKPOINT")
            else None
        )
        profile = os.environ.get("ROBOTURK_CARRY_PROFILE")
        first_touch_steps = None
        if profile:
            from roboturk.closing_press import ClosingCarry

            settings = json.loads(Path(profile).read_text())
            first_touch_steps = settings.pop("first_touch_steps", None)
            shooter = ClosingCarry(shooter, **settings)
        if first_touch_steps is None:
            policy = TeamCheckpoint(checkpoint, cfg) if checkpoint else TransferTeacher()
        else:
            from roboturk.closing_press import FirstTouchTransfer

            policy = FirstTouchTransfer(checkpoint, cfg, shooter, first_touch_steps)
        episodes, seed = (
            int(os.environ.get("ROBOTURK_EPISODES", "200")),
            int(os.environ.get("ROBOTURK_SEED", "450000")),
        )
        if mode in ("team-demo", "team-closing-demo"):
            if not checkpoint or shooter is None:
                raise ValueError("Demo requires both learned transfer and scoring checkpoints")
            from team_demo import record

            result = record(supervisor, env, policy, shooter, output, episodes, seed)
        else:
            result = evaluate_team(env, policy, episodes, seed, shooter)
        if profile:
            result["carry_profile"] = json.loads(Path(profile).read_text())
            result["carry_profile_sha256"] = hashlib.sha256(Path(profile).read_bytes()).hexdigest()
        result["policy"] = checkpoint or "analytic_demonstration_teacher"
        result["shooter"] = os.environ.get("ROBOTURK_SHOOT_CHECKPOINT")
        for label, path in [("checkpoint", checkpoint), ("shooting_checkpoint", result["shooter"])]:
            if path:
                result[label + "_manifest_sha256"] = hashlib.sha256(
                    Path(path).with_suffix(".json").read_bytes()
                ).hexdigest()
                result[label + "_sha256"] = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({k: v for k, v in result.items() if k != "episodes"}), flush=True)
    env.close()
    supervisor.simulationQuit(0)
