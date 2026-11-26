"""Single-owner continuous native experiments."""

import os
from pathlib import Path
import traceback
import torch
from controller import Supervisor
from roboturk.model import Config
from roboturk.webots_backend import WebotsBackend
from roboturk.continuous_training import collect, evaluate, fit, load, descriptor
from roboturk.self_play_training import write


def main():
    torch.set_num_threads(1)
    supervisor = Supervisor()
    backend = WebotsBackend(supervisor, Config())
    env = os.environ
    mode = env["CONTINUOUS_MODE"]
    out = Path(env["CONTINUOUS_OUTPUT"])
    seed, count = int(env["CONTINUOUS_SEED"]), int(env["CONTINUOUS_EPISODES"])
    duration = float(env["CONTINUOUS_DURATION"])
    try:
        source = env.get("CONTINUOUS_CHECKPOINT")
        if mode == "pilot":
            result = evaluate(backend, None, None, count, seed, duration)
        elif mode == "collect":
            model = load(source) if source else None
            collect(backend, out, count, seed, duration, model, float(env["CONTINUOUS_MIX"]))
            result = None
        elif mode == "fit":
            fit(out, seed, int(env["CONTINUOUS_EPOCHS"]), source)
            result = None
        elif mode == "train":
            from roboturk.continuous_selfplay import train

            train(
                backend,
                out,
                seed,
                int(env["CONTINUOUS_ROUNDS"]),
                int(env["CONTINUOUS_STEPS"]),
                duration,
            )
            result = None
        elif mode in ("evaluate", "demo"):
            blue = load(source)
            opponent = env.get("CONTINUOUS_OPPONENT", source)
            yellow = load(opponent)
            if mode == "demo":
                from continuous_demo import record

                result = record(supervisor, backend, blue, yellow, out, count, seed, duration)
            else:
                result = evaluate(backend, blue, yellow, count, seed, duration)
            result["models"] = {
                "blue": descriptor(source, blue),
                "yellow": descriptor(opponent, yellow),
            }
        else:
            raise ValueError("Continuous mode not implemented yet: " + mode)
        if result:
            write(out.with_suffix(".json"), result)
        supervisor.simulationQuit(0)
    except Exception:
        traceback.print_exc()
        write(out.with_suffix(".error.json"), {"error": traceback.format_exc()})
        supervisor.simulationQuit(1)


if __name__ == "__main__":
    main()
