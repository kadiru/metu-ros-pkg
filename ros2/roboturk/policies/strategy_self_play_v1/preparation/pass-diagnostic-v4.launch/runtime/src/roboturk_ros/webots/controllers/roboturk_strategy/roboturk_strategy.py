"""One native simulator owns integrated option training and evaluation."""

import os
from pathlib import Path
import traceback
import torch
from controller import Supervisor
from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.continuous_safety import filter_commands, CONTRACT as SAFETY
from roboturk.strategy_options import MotorBank
from roboturk.strategy_training import evaluate, collect, load, write


def main():
    torch.set_num_threads(1)
    supervisor = Supervisor()
    backend = ContinuousWebotsBackend(supervisor)
    backend.command_filter = filter_commands
    backend.safety_contract = SAFETY
    args = os.environ
    root = Path(__file__).resolve().parents[5]
    bank = MotorBank(root)
    out = Path(args["STRATEGY_OUTPUT"])
    mode = args["STRATEGY_MODE"]
    if args.get("STRATEGY_DIAGNOSTIC_PASS") == "True":
        assert mode in ("pilot", "collect")
        from roboturk.strategy_pass import DiagnosticPassBank

        bank = DiagnosticPassBank(bank)
    count, seed = int(args["STRATEGY_EPISODES"]), int(args["STRATEGY_SEED"])
    duration = float(args["STRATEGY_DURATION"])
    try:
        checkpoint = args.get("STRATEGY_CHECKPOINT")
        if mode == "pilot":
            result = evaluate(backend, bank, None, count, seed, duration)
        elif mode == "collect":
            collect(
                backend,
                bank,
                out,
                count,
                seed,
                duration,
                load(checkpoint) if checkpoint else None,
                float(args["STRATEGY_MIX"]),
            )
            result = None
        elif mode == "prepare":
            from roboturk.strategy_training import prepare

            prepare(backend, bank, out)
            result = None
        elif mode == "refine":
            from roboturk.strategy_training import refine

            refine(backend, bank, out)
            result = None
        elif mode == "adapt":
            from roboturk.strategy_adapt import adapt

            adapt(backend, bank, out)
            result = None
        elif mode == "train":
            from roboturk.strategy_selfplay import train

            train(
                backend,
                bank,
                out,
                checkpoint,
                int(args["STRATEGY_TRAINING_SEED"]),
                int(args["STRATEGY_ROUNDS"]),
                int(args["STRATEGY_STEPS"]),
                duration,
            )
            result = None
        else:
            models = [load(checkpoint), load(args.get("STRATEGY_OPPONENT", checkpoint))]
            if mode == "demo":
                from strategy_demo import record

                result = record(supervisor, backend, bank, models, out, count, seed, duration)
            else:
                result = evaluate(backend, bank, models, count, seed, duration)
            from roboturk.strategy_training import digest

            result["selectors"] = [
                {
                    "path": str(p),
                    "sha256": digest(p),
                    "metadata_sha256": digest(Path(p).with_suffix(".json")),
                }
                for p in [checkpoint, args.get("STRATEGY_OPPONENT", checkpoint)]
            ]
        if result is not None:
            write(out.with_suffix(".json"), result)
        supervisor.simulationQuit(0)
    except Exception:
        traceback.print_exc()
        write(out.with_suffix(".error.json"), {"error": traceback.format_exc()})
        supervisor.simulationQuit(1)


if __name__ == "__main__":
    main()
