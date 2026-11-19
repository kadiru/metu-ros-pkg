"""Native learned-opponent training and frozen cross-play entry points."""

import hashlib
import json
import os
from pathlib import Path
import torch
from roboturk.defended import DefendedCheckpoint
from roboturk.keeper import FinishCheckpoint
from roboturk.learned_opponents import OpponentController, LearnedOpponentPlayEnv
from roboturk.team import evaluate_team


def run(supervisor, backend, cfg, mode):
    torch.set_num_threads(1)
    output = Path(os.environ["ROBOTURK_OUTPUT"])
    output.parent.mkdir(parents=True, exist_ok=True)
    episodes, seed = int(os.environ["ROBOTURK_EPISODES"]), int(os.environ["ROBOTURK_SEED"])
    if mode == "opponents-train":
        from roboturk.opponent_training import train_opponents

        train_opponents(
            backend,
            cfg,
            output,
            DefendedCheckpoint(os.environ["ROBOTURK_CHECKPOINT"], cfg),
            FinishCheckpoint(os.environ["ROBOTURK_SHOOT_CHECKPOINT"], cfg),
            int(os.environ["ROBOTURK_STEPS"]),
            seed,
        )
    elif mode == "opponents-challenge":
        from roboturk.opponent_training import OpponentChallengeEnv, ChallengeTeacher, ActionModel
        from roboturk.micro_training import evaluate, Actor

        role = os.environ.get("ROBOTURK_OPPONENT_ROLE", "keeper")
        env = OpponentChallengeEnv(backend, cfg, role)
        path = os.environ.get("ROBOTURK_CHECKPOINT")
        stationary = os.environ.get("ROBOTURK_CHALLENGE_CONTROL") == "stationary"
        if stationary and path:
            raise ValueError("Stationary baseline cannot also specify a checkpoint")
        actor = (
            Actor(ActionModel())
            if stationary
            else Actor(OpponentController(role, cfg, path=path).model)
            if path
            else ChallengeTeacher()
        )
        result = evaluate(env, actor, episodes, seed)
        result.update(
            actor_source="stationary"
            if stationary
            else "neural_checkpoint"
            if path
            else "analytic_teacher",
            backend=backend.name,
            checkpoint_sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest() if path else None,
        )
        output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
    else:
        defender = OpponentController(
            "defender", cfg, path=os.environ["ROBOTURK_DEFENDER_CHECKPOINT"]
        )
        keeper = OpponentController("keeper", cfg, path=os.environ["ROBOTURK_KEEPER_CHECKPOINT"])
        env = LearnedOpponentPlayEnv(cfg, backend, defender, keeper)
        transfer_path, finish_path = (
            os.environ["ROBOTURK_CHECKPOINT"],
            os.environ["ROBOTURK_SHOOT_CHECKPOINT"],
        )
        transfer, shooter = (
            DefendedCheckpoint(transfer_path, cfg),
            FinishCheckpoint(finish_path, cfg),
        )
        if mode == "opponents-attack-train":
            from roboturk.adversarial_training import train_attackers

            train_attackers(
                backend,
                cfg,
                output,
                defender,
                keeper,
                transfer_path,
                finish_path,
                int(os.environ["ROBOTURK_STEPS"]),
                seed,
            )
        else:
            if mode == "opponents-demo":
                from opponent_demo import record

                result = record(supervisor, env, transfer, shooter, output, episodes, seed)
            else:
                result = evaluate_team(env, transfer, episodes, seed, shooter)
            result.update(
                backend=env.backend.name,
                opponent_policy_fallback=False,
                checkpoint_sha256=hashlib.sha256(Path(transfer_path).read_bytes()).hexdigest(),
                shooting_checkpoint_sha256=hashlib.sha256(
                    Path(finish_path).read_bytes()
                ).hexdigest(),
                defender_checkpoint_sha256=defender.sha256,
                keeper_checkpoint_sha256=keeper.sha256,
            )
            output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps({k: v for k, v in result.items() if k != "episodes"}), flush=True)
    backend.close()
    supervisor.simulationQuit(0)
