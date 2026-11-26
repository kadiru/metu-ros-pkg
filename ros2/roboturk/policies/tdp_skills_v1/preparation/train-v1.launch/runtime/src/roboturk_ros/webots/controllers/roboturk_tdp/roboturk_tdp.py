"""Isolated native controller for new TDP-inspired skill options."""

import json
import os
from pathlib import Path
import traceback
from controller import Supervisor
from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.tdp_options import BallOptionEnv, BallOptionTeacher, KINDS


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, separators=(",", ":")) + "\n")


def evaluate(env, actor, count, seed, recorder=None):
    episodes = []
    for i in range(count):
        env.reset(seed=seed + i)
        if recorder:
            recorder.begin(i, env)
        while True:
            action = actor.action(env)
            _, _, done, truncated, info = env.step(action)
            if recorder:
                recorder.step(env, info)
            if done or truncated:
                episodes.append(info)
                break
        if (i + 1) % 10 == 0:
            print(env.kind, i + 1, count, sum(e["is_success"] for e in episodes), flush=True)
    return {
        "kind": env.kind,
        "control": "analytic_teacher" if isinstance(actor, BallOptionTeacher) else "neural",
        "backend": env.backend.name,
        "episodes": episodes,
        "successes": sum(e["is_success"] for e in episodes),
        "count": count,
    }


def main():
    supervisor = Supervisor()
    backend = ContinuousWebotsBackend(supervisor)
    args = os.environ
    out = Path(args["TDP_OUTPUT"])
    mode = args["TDP_MODE"]
    kind = args["TDP_KIND"]
    try:
        kinds = KINDS if kind == "all" else (kind,)
        for k in kinds:
            env = BallOptionEnv(backend, k)
            destination = out / k if kind == "all" else out
            seed = int(args["TDP_SEED"]) + (200000 * KINDS.index(k) if kind == "all" else 0)
            if mode == "pilot":
                result = evaluate(env, BallOptionTeacher(), int(args["TDP_EPISODES"]), seed)
            else:
                from roboturk.tdp_training import train, OptionActor, load

                if mode == "train":
                    train(
                        env,
                        destination,
                        int(args["TDP_STEPS"]),
                        711 + 11 * KINDS.index(k),
                        evaluate,
                    )
                    continue
                source = (
                    Path(args["TDP_CHECKPOINT"]) / (k + "/policy.zip")
                    if kind == "all"
                    else Path(args["TDP_CHECKPOINT"])
                )
                actor = OptionActor(load(source, env))
                if mode == "demo":
                    from tdp_demo import record

                    result = record(
                        supervisor,
                        env,
                        actor,
                        destination,
                        int(args["TDP_EPISODES"]),
                        seed,
                        evaluate,
                    )
                else:
                    result = evaluate(env, actor, int(args["TDP_EPISODES"]), seed)
                from roboturk.tdp_training import descriptor

                result["checkpoint"] = descriptor(source, actor.model)
            write(destination.with_suffix(".json"), result)
            print(k, result["successes"], result["count"], flush=True)
        supervisor.simulationQuit(0)
    except Exception:
        traceback.print_exc()
        write(out.with_suffix(".error.json"), {"error": traceback.format_exc()})
        supervisor.simulationQuit(1)


if __name__ == "__main__":
    main()
