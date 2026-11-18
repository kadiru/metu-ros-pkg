import json
import hashlib
import os
from pathlib import Path
from roboturk.capture import CaptureEnv, CaptureTeacher, evaluate_capture
from roboturk.behavior import CheckpointPolicy
from roboturk.model import Skill


def run(supervisor, backend, cfg, mode):
    env = CaptureEnv(cfg, backend)
    output = Path(os.environ.get("ROBOTURK_OUTPUT", "runs/capture/result"))
    output.parent.mkdir(parents=True, exist_ok=True)
    if mode == "capture-train":
        from roboturk.capture_training import train_capture

        train_capture(
            env,
            output,
            int(os.environ.get("ROBOTURK_STEPS", "131072")),
            int(os.environ.get("ROBOTURK_SEED", "7")),
            int(os.environ.get("ROBOTURK_DEMOS", "300")),
            int(os.environ.get("ROBOTURK_DAGGER", "150")),
        )
    else:
        checkpoint = os.environ.get("ROBOTURK_CHECKPOINT")
        policy = CheckpointPolicy(checkpoint, Skill.CHASE, cfg) if checkpoint else CaptureTeacher()
        if mode == "capture-demo":
            if not checkpoint:
                raise ValueError("A learned checkpoint is required for capture-demo")
            from capture_demo import record

            result = record(
                supervisor,
                env,
                policy,
                output,
                int(os.environ.get("ROBOTURK_EPISODES", "6")),
                int(os.environ.get("ROBOTURK_SEED", "60000")),
            )
        else:
            result = evaluate_capture(
                env,
                policy,
                int(os.environ.get("ROBOTURK_EPISODES", "200")),
                int(os.environ.get("ROBOTURK_SEED", "50000")),
            )
        result["policy"] = checkpoint or "analytic_demonstration_teacher"
        if checkpoint:
            result["checkpoint_sha256"] = hashlib.sha256(Path(checkpoint).read_bytes()).hexdigest()
        output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({k: v for k, v in result.items() if k != "episodes"}), flush=True)
    env.close()
    supervisor.simulationQuit(0)
