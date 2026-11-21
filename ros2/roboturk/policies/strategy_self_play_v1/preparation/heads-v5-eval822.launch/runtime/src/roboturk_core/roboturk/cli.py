import argparse
from dataclasses import asdict
import json
from pathlib import Path

from .behavior import AnalyticPolicy, CheckpointPolicy
from .contract import CONTRACT_VERSION, encode_action
from .env import SkillEnv
from .model import Config, Skill


def evaluate(env, policy, episodes=10, seed=1000, trace=False):
    records = []
    for episode in range(episodes):
        env.reset(seed=seed + episode)
        total = 0.0
        samples = []
        while True:
            command = policy.command(env.world, env.key, env.intent, env.cfg)
            _, reward, terminated, truncated, info = env.step(encode_action(command, env.cfg))
            total += reward
            if trace and (env.steps % 10 == 0 or terminated or truncated):
                samples.append(
                    {
                        "step": env.steps,
                        "ball": asdict(env.world.ball),
                        "robots": [asdict(r) for r in env.world.robots],
                    }
                )
            if terminated or truncated:
                record = {**info, "seed": seed + episode, "return": round(total, 4)}
                if trace:
                    record["trace"] = samples
                records.append(record)
                break
    return {"episodes": records, "success_rate": sum(r["is_success"] for r in records) / episodes}


def train(env, steps, output, seed=0):
    from stable_baselines3 import PPO
    from stable_baselines3.common.monitor import Monitor
    import torch

    torch.set_num_threads(1)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    model = PPO(
        "MlpPolicy",
        Monitor(env),
        seed=seed,
        device="cpu",
        n_steps=256,
        batch_size=64,
        verbose=1,
        policy_kwargs={"net_arch": [64, 64]},
    )
    model.learn(total_timesteps=steps)
    model.save(output)
    output.with_suffix(".json").write_text(
        json.dumps(
            {
                "contract": CONTRACT_VERSION,
                "skill": env.skill.value,
                "config": asdict(env.cfg),
                "backend": env.backend.name,
                "seed": seed,
                "requested_steps": steps,
                "actual_steps": model.num_timesteps,
                "validated_for_hardware": False,
            },
            indent=2,
        )
        + "\n"
    )


def main():
    parser = argparse.ArgumentParser(
        description="RoboTurk synthetic CPU smoke tests; use Webots controller for physics"
    )
    parser.add_argument("mode", choices=["evaluate", "train"])
    parser.add_argument(
        "--skill", choices=[s.value for s in Skill if s != Skill.HOLD], default="chase"
    )
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--steps", type=int, default=2048)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--checkpoint")
    parser.add_argument("--output", default="runs/synthetic_smoke/policy")
    args = parser.parse_args()
    if args.episodes < 1 or args.steps < 1:
        parser.error("episodes and steps must be positive")
    cfg = Config()
    env = SkillEnv(args.skill, cfg)
    try:
        if args.mode == "train":
            train(env, args.steps, args.output, args.seed)
        else:
            policy = (
                CheckpointPolicy(args.checkpoint, env.skill, cfg)
                if args.checkpoint
                else AnalyticPolicy()
            )
            print(json.dumps(evaluate(env, policy, args.episodes, args.seed), indent=2))
    finally:
        env.close()


if __name__ == "__main__":
    main()
