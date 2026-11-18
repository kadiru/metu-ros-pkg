"""Run inside Webots: ROS bridge, analytic evaluation, or bounded PPO training."""

import json
import os
from pathlib import Path

from controller import Supervisor

from roboturk.model import Config, Skill
from roboturk.webots_backend import WebotsBackend


def main():
    supervisor = Supervisor()
    cfg = Config()
    backend = WebotsBackend(supervisor, cfg)
    mode = os.environ.get("ROBOTURK_MODE", "evaluate")
    if mode.startswith("capture-"):
        from capture_runner import run

        try:
            run(supervisor, backend, cfg, mode)
        except Exception:
            import traceback

            traceback.print_exc()
            supervisor.simulationQuit(1)
        return
    if mode == "ros":
        from roboturk_ros.bridge import run

        run(supervisor, backend, cfg)
        return
    from roboturk.behavior import AnalyticPolicy, CheckpointPolicy
    from roboturk.cli import evaluate, train
    from roboturk.env import SkillEnv

    env = SkillEnv(os.environ.get("ROBOTURK_SKILL", "chase"), cfg, backend)
    output = Path(os.environ.get("ROBOTURK_OUTPUT", "runs/webots/result"))
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        if mode == "suite":
            results = {}
            for skill in Skill:
                if skill == Skill.HOLD:
                    continue
                task = SkillEnv(skill, cfg, backend)
                results[skill.value] = evaluate(
                    task,
                    AnalyticPolicy(),
                    int(os.environ.get("ROBOTURK_EPISODES", "10")),
                    int(os.environ.get("ROBOTURK_SEED", "1000")),
                )
            output.with_suffix(".json").write_text(json.dumps(results, indent=2) + "\n")
            print(json.dumps(results), flush=True)
        elif mode == "train":
            train(
                env,
                int(os.environ.get("ROBOTURK_STEPS", "2048")),
                output,
                int(os.environ.get("ROBOTURK_SEED", "0")),
            )
        elif mode == "evaluate":
            checkpoint = os.environ.get("ROBOTURK_CHECKPOINT")
            policy = (
                CheckpointPolicy(checkpoint, env.skill, cfg) if checkpoint else AnalyticPolicy()
            )
            result = evaluate(
                env,
                policy,
                int(os.environ.get("ROBOTURK_EPISODES", "10")),
                int(os.environ.get("ROBOTURK_SEED", "1000")),
                trace=os.environ.get("ROBOTURK_TRACE") == "1",
            )
            output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result), flush=True)
        else:
            raise ValueError(f"Unknown ROBOTURK_MODE: {mode}")
    except Exception:
        supervisor.simulationQuit(1)
        raise
    finally:
        env.close()
    supervisor.simulationQuit(0)


if __name__ == "__main__":
    main()
