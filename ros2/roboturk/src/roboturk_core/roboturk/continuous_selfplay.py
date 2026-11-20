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

REWARD = {
    "goal": 12.0,
    "controlled_carry_meter": 0.1,
    "acquisition": 0.05,
    "reception": 0.2,
    "contact_event": 3.0,
    "contact_physics_sample": 0.02,
    "outside_physics_sample": 0.1,
    "stable_possession_control_tick": 0.002,
    "loose_ball_progress_meter": 0.1,
}


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
        self.role_order = []

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        match_seed = int(self.rng.integers(4300000, 4400000))
        if not self.role_order:
            self.role_order = self.rng.permutation(4).tolist()
        self.key = KEYS[self.role_order.pop()]
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
        reward = REWARD["goal"] * (
            (score(after, team) - score(before, team))
            - (score(after, enemy) - score(before, enemy))
        )
        reward += REWARD["controlled_carry_meter"] * (
            after["controlled_carry"].get(team, 0) - before["controlled_carry"].get(team, 0)
        )
        reward += REWARD["acquisition"] * (
            after["acquisitions"].get(team, 0) - before["acquisitions"].get(team, 0)
        )
        reward += REWARD["reception"] * (
            after["receptions"].get(team, 0) - before["receptions"].get(team, 0)
        )
        reward -= REWARD["contact_event"] * (after["contact_events"] - before["contact_events"])
        reward -= REWARD["contact_physics_sample"] * (
            after["contact_checks"] - before["contact_checks"]
        )
        reward -= REWARD["outside_physics_sample"] * (
            after["outside_checks"] - before["outside_checks"]
        )
        if m.current_owner:
            reward += REWARD["stable_possession_control_tick"] * (
                1 if m.current_owner.team == team else -1
            )
        else:
            distance = min(
                np.hypot(m.world.robot(k).x - m.world.ball.x, m.world.robot(k).y - m.world.ball.y)
                for k in KEYS
                if k.team == team
            )
            reward += REWARD["loose_ball_progress_meter"] * (before_distance - distance)
        done = m.finished
        if done:
            self.records.append(
                {**after, "learner": name(self.key), "opponent_index": self.pool_index}
            )
        else:
            m.restart_if_needed()
        return observation(m.world, self.key, m.memory), float(reward), done, False, {}


def train(backend, output, seed, rounds=2, steps=32768, duration=45, decisions_only=False):
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
    modes = np.argmax(y[:, :7], axis=1)
    groups += [
        np.flatnonzero(y[np.arange(len(y)), 10 + 4 * modes] > 0),
        np.flatnonzero((np.abs(x[:, 0]) * 4.5 > 3.8) | (np.abs(x[:, 1]) * 3 > 2.3)),
        np.flatnonzero(np.min(x[:, [54, 58, 62]], axis=1) * 5 < 0.5),
        np.flatnonzero(np.abs(x[:, 121]) < 0.5),
    ]
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
        if decisions_only:
            freeze_motion(model)
        model.prepare_anchor(x, training_seed, groups)
        model.anchor_optimizer = torch.optim.Adam(model.actor_parameters(), lr=3e-6)
        model.actor_mse_limit = 3e-4
        model.actor_delta_limit = 0.15
        with torch.no_grad():
            model.policy.log_std.fill_(-5)
            model.policy.log_std[:7].fill_(-0.8)
            model.policy.log_std[10::4].fill_(-1.5)
            if decisions_only:
                model.policy.log_std[7:].fill_(-12)
        before = actor_hash(model)
        decision_before = decision_head_hash(model)
        motion_before = motion_hash(model)
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
        with torch.no_grad():
            probe = model.policy.get_distribution(
                model.anchor_inputs[model.anchor_probe]
            ).distribution.mean
            mode_changes = int(
                (
                    probe[:, :7].argmax(1) != model.anchor_targets[model.anchor_probe, :7].argmax(1)
                ).sum()
            )
        if decisions_only and motion_hash(model) != motion_before:
            raise RuntimeError("Frozen motion parameters changed")
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
            "reward": REWARD,
            "role_sampling": "shuffled cycles of all four robot/color assignments",
            "anchor_group_sizes": [len(g) for g in groups],
            "decisions_only": decisions_only,
            "motion_before": motion_before,
            "motion_after": motion_hash(model),
            "deterministic_mode_probe_changes": mode_changes,
            "mode_probe_rows": len(model.anchor_probe),
            "backend_contract": getattr(backend, "contract", "legacy_ordered_kicks"),
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


def motion_hash(model):
    h = hashlib.sha256()
    for tensor in list(model.policy.mlp_extractor.policy_net.state_dict().values()) + [
        model.policy.action_net.weight[7:],
        model.policy.action_net.bias[7:],
    ]:
        h.update(tensor.detach().cpu().contiguous().numpy().tobytes())
    return h.hexdigest()


def freeze_motion(model):
    """Learn behavior readouts while preserving every neural motor function."""
    model.policy.mlp_extractor.policy_net.requires_grad_(False)
    for parameter in (
        model.policy.action_net.weight,
        model.policy.action_net.bias,
        model.policy.log_std,
    ):
        mask = torch.zeros_like(parameter)
        mask[:7] = 1
        parameter.register_hook(lambda grad, mask=mask: grad * mask)
    # Old Adam momentum could otherwise move rows with newly masked gradients.
    model.policy.optimizer.state.clear()
