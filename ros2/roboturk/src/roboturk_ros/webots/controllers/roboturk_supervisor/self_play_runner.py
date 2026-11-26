"""Single-owner native self-play experiments and frozen evaluation."""

import json
import os
from pathlib import Path
import torch
from roboturk.self_play import (
    MobileOpponent,
    MobilePlayEnv,
    MobileTransfer,
    MobileFinish,
    outcomes,
    digest,
)
from roboturk.self_play_training import bootstrap, train_league, collect_full, write
from roboturk.team import evaluate_team


def run(supervisor, backend, cfg, mode):
    torch.set_num_threads(1)
    output = Path(os.environ["ROBOTURK_OUTPUT"])
    episodes, seed = int(os.environ["ROBOTURK_EPISODES"]), int(os.environ["ROBOTURK_SEED"])
    if mode == "selfplay-probe":
        data, result = collect_full(backend, cfg, episodes, seed)
        result["sample_counts"] = {r: len(x) for r, (x, _) in data.items()}
        write(output.with_suffix(".json"), result)
        print(outcomes(result), flush=True)
    elif mode == "selfplay-bootstrap":
        sources = {
            "transfer": os.environ["ROBOTURK_CHECKPOINT"],
            "finish": os.environ["ROBOTURK_SHOOT_CHECKPOINT"],
            "defender": os.environ["ROBOTURK_DEFENDER_CHECKPOINT"],
            "keeper": os.environ["ROBOTURK_KEEPER_CHECKPOINT"],
        }
        bootstrap(backend, cfg, output, sources, seed)
    elif mode == "selfplay-panel":
        from roboturk.self_play_panel import cross_play

        cross_play(backend, cfg, output, episodes, seed)
    elif mode == "selfplay-train":
        train_league(
            backend,
            cfg,
            output,
            rounds=int(os.environ.get("ROBOTURK_ROUNDS", "2")),
            steps=int(os.environ["ROBOTURK_STEPS"]),
            seed=seed,
            prepared_bundle=os.environ.get("ROBOTURK_PREPARED_BUNDLE"),
        )
    else:
        paths = {
            "transfer": os.environ["ROBOTURK_CHECKPOINT"],
            "finish": os.environ["ROBOTURK_SHOOT_CHECKPOINT"],
            "defender": os.environ["ROBOTURK_DEFENDER_CHECKPOINT"],
            "keeper": os.environ["ROBOTURK_KEEPER_CHECKPOINT"],
        }
        defender = MobileOpponent("defender", cfg, path=paths["defender"])
        keeper = MobileOpponent("keeper", cfg, path=paths["keeper"])
        env = MobilePlayEnv(cfg, backend, defender, keeper)
        transfer, finish = (
            MobileTransfer(paths["transfer"], cfg),
            MobileFinish(paths["finish"], cfg),
        )
        if mode == "selfplay-demo":
            from self_play_demo import record

            result = record(supervisor, env, transfer, finish, output, episodes, seed)
        elif mode == "selfplay-evaluate":
            result = evaluate_team(env, transfer, episodes, seed, finish)
        else:
            raise ValueError("Unknown self-play mode")
        result.update(
            backend=env.backend.name,
            opponent_policy_fallback=False,
            models={r: {"path": str(p), "sha256": digest(p)} for r, p in paths.items()},
            outcome_summary=outcomes(result),
        )
        write(output.with_suffix(".json"), result)
        print(json.dumps(result["outcome_summary"]), flush=True)
    backend.close()
    supervisor.simulationQuit(0)
