"""Historical self-play; one randomized robot learns while teammates/opponents are frozen."""

from pathlib import Path
from collections import Counter
import hashlib
import time
import json
import numpy as np
import torch
import gymnasium as gym
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from .continuous import ContinuousMatch, KEYS, observation, name
from .continuous_training import ShapeEnv, load, save, evaluate, digest
from .self_play_training import SelfPlayPPO, actor_hash, write


class RobotMatchEnv(gym.Env):
    observation_space = ShapeEnv.observation_space
    action_space = ShapeEnv.action_space

    def __init__(self, backend, history, teammate, seed, duration=45):
        self.match = ContinuousMatch(backend, duration)
        self.history = history
        self.opponents = [load(x["path"]) for x in history]
        self.teammate = load(teammate["path"])
        self.rng = np.random.default_rng(seed)
        self.records = []
        self.resets = []
        self.sampled_modes = Counter()
        self.changed_choices = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        match_seed = int(self.rng.integers(4300000, 4400000))
        self.key = KEYS[int(self.rng.integers(4))]
        self.pool_index = int(self.rng.integers(len(self.opponents)))
        self.match.reset(match_seed)
        self.resets.append(
            {
                "seed": match_seed,
                "learner": name(self.key),
                "opponent_index": self.pool_index,
                "opponent_sha256": self.history[self.pool_index]["sha256"],
            }
        )
        return observation(self.match.world, self.key, self.match.memory), {}

    def step(self, action):
        m = self.match
        before = m.result()
        before_distance = min(
            np.hypot(m.world.robot(k).x - m.world.ball.x, m.world.robot(k).y - m.world.ball.y)
            for k in KEYS
            if k.team == self.key.team
        )
        obs = m.observations()
        actions = np.empty((4, self.action_space.shape[0]), np.float32)
        for team in ("blue", "yellow"):
            ids = [i for i, k in enumerate(KEYS) if k.team == team]
            model = self.teammate if team == self.key.team else self.opponents[self.pool_index]
            actions[ids] = model.predict(obs[ids], deterministic=True)[0]
        learner_index = KEYS.index(self.key)
        choice = int(np.argmax(action[:7]))
        self.sampled_modes[choice] += 1
        self.changed_choices += int(choice != int(np.argmax(actions[learner_index, :7])))
        actions[learner_index] = action
        m.step(actions)
        team = self.key.team
        enemy = "yellow" if team == "blue" else "blue"

        def score(d, t):
            return d["score"].get(t, 0)

        after = m.result()
        reward = 12 * (
            (score(after, team) - score(before, team))
            - (score(after, enemy) - score(before, enemy))
        )
        reward += 0.4 * (
            after["controlled_carry"].get(team, 0) - before["controlled_carry"].get(team, 0)
        )
        reward += 0.3 * (after["acquisitions"].get(team, 0) - before["acquisitions"].get(team, 0))
        reward += 0.8 * (after["receptions"].get(team, 0) - before["receptions"].get(team, 0))
        reward -= 3 * (after["contact_events"] - before["contact_events"])
        reward -= 0.1 * (after["outside_checks"] - before["outside_checks"])
        if m.current_owner:
            reward += 0.01 if m.current_owner.team == team else -0.01
        else:
            distance = min(
                np.hypot(m.world.robot(k).x - m.world.ball.x, m.world.robot(k).y - m.world.ball.y)
                for k in KEYS
                if k.team == team
            )
            reward += 0.1 * (before_distance - distance)
        done = m.finished
        if done:
            self.records.append(
                {**after, "learner": name(self.key), "opponent_index": self.pool_index}
            )
        else:
            m.restart_if_needed()
        return observation(m.world, self.key, m.memory), float(reward), done, False, {}


def train(backend, output, seed, rounds=2, steps=32768, duration=45):
    out = Path(output)
    league_dir = out / f"seed{seed}"
    progress = league_dir / "league.json"
    initial = league_dir / "bootstrap.zip"
    if progress.exists():
        ledger = json.loads(progress.read_text())
        history = ledger["snapshots"]
        updates = ledger["updates"]
    else:
        model = load(initial)
        history = [
            {"path": str(initial), "sha256": digest(initial), "actor_sha256": actor_hash(model)}
        ]
        updates = []
    data = np.load(out / "data.npz")
    x, y = data["observations"], data["actions"]
    groups = [np.flatnonzero(np.argmax(y[:, :7], axis=1) == i) for i in range(7)]
    groups = [g for g in groups if len(g)]
    for generation in range(len(history), rounds + 1):
        stage = league_dir / f"round{generation}"
        if stage.exists():
            raise RuntimeError("Incomplete round exists; retain and investigate before resume")
        stage.mkdir()
        training_seed = seed + generation * 101
        env = RobotMatchEnv(backend, history, history[-1], training_seed, duration)
        model = SelfPlayPPO.load(
            history[-1]["path"], env=Monitor(env), device="cpu", learning_rate=3e-6, gamma=0.999
        )
        model.set_random_seed(training_seed)
        model.prepare_anchor(x, training_seed, groups)
        model.anchor_optimizer = torch.optim.Adam(model.actor_parameters(), lr=3e-6)
        model.actor_mse_limit = 3e-4
        model.actor_delta_limit = 0.15
        with torch.no_grad():
            model.policy.log_std.fill_(-5)
            model.policy.log_std[:7].fill_(-0.8)
            model.policy.log_std[10::4].fill_(-1.5)
        before = actor_hash(model)
        decision_before = decision_head_hash(model)
        started = time.monotonic()

        class Progress(BaseCallback):
            def _on_step(self):
                if self.num_timesteps % 8192 == 0:
                    print(
                        "Continuous PPO",
                        seed,
                        generation,
                        self.num_timesteps,
                        "seconds",
                        round(time.monotonic() - started, 1),
                        flush=True,
                    )
                return True

        model.learn(total_timesteps=steps, reset_num_timesteps=True, callback=Progress())
        metadata = {
            "generation": generation,
            "seed": seed,
            "training_seed": training_seed,
            "steps": model.num_timesteps,
            "backend": backend.name,
            "actor_before": before,
            "actor_after": actor_hash(model),
            "decision_head_before": decision_before,
            "decision_head_after": decision_head_hash(model),
            "sampled_modes": dict(env.sampled_modes),
            "choices_different_from_teammate_policy": env.changed_choices,
            "opponent_pool": history.copy(),
            "teammate": history[-1],
            "accepted_updates": model.anchor_updates - model.anchor_rejections,
            "attempted_updates": model.anchor_updates,
            "learning_rate": 3e-6,
            "anchor_mse_limit": 3e-4,
            "anchor_max_delta": 0.15,
            "duration": duration,
            "training_seconds": time.monotonic() - started,
            "training_seed_range": [4300000, 4399999],
            "reset_count": len(env.resets),
            "data_sha256": digest(out / "data.npz"),
            "log_std": model.policy.log_std.detach().tolist(),
        }
        saved = save(model, stage / "policy.zip", metadata)
        write(stage / "training.json", metadata)
        write(stage / "resets.json", env.resets)
        write(stage / "rollouts.json", env.records)
        write(stage / "updates.json", model.anchor_log)
        if before == actor_hash(model):
            raise RuntimeError("No actual policy update; retained failed round")
        history.append(saved)
        updates.append(metadata)
        write(progress, {"training_seed": seed, "snapshots": history, "updates": updates})
        result = evaluate(backend, model, model, 6, 4400000, duration, trace=False)
        result["models"] = {"blue": saved, "yellow": saved}
        write(stage / "development.json", result)
    return history


def decision_head_hash(model):
    h = hashlib.sha256()
    for tensor in (model.policy.action_net.weight[:7], model.policy.action_net.bias[:7]):
        h.update(tensor.detach().cpu().contiguous().numpy().tobytes())
    return h.hexdigest()
