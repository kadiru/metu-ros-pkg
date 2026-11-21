"""Historical self-play learns ten-way strategic choices against frozen snapshots."""

from collections import Counter
from pathlib import Path
import hashlib
import json
import time
import numpy as np
import torch
import gymnasium as gym
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from .continuous import KEYS, name
from .strategy_options import StrategyMatch, MODES
from .strategy_training import ShapeEnv, load, save, write, digest, evaluate, summarize
from .self_play_training import SelfPlayPPO, actor_hash

REWARD = {
    "goal_difference": 12.0,
    "receive_stop": 1.0,
    "redirect_kick": 0.4,
    "redirect_goal": 2.0,
    "clearance_complete": 1.0,
    "acquisition": 0.05,
    "carry_meter": 0.1,
    "contact_event": 3.0,
    "contact_sample": 0.02,
    "outside_sample": 0.1,
    "safety_correction": 0.01,
    "possession_tick": 0.002,
    "loose_progress_meter": 0.1,
}


def target_hash(model):
    h = hashlib.sha256()
    tensors = list(model.policy.mlp_extractor.policy_net.state_dict().values())
    tensors += [model.policy.action_net.weight[10:], model.policy.action_net.bias[10:]]
    for tensor in tensors:
        h.update(tensor.detach().cpu().contiguous().numpy().tobytes())
    return h.hexdigest()


def freeze_targets(model):
    model.policy.mlp_extractor.policy_net.requires_grad_(False)
    for p in (model.policy.action_net.weight, model.policy.action_net.bias, model.policy.log_std):
        mask = torch.zeros_like(p)
        mask[:10] = 1
        p.register_hook(lambda grad, mask=mask: grad * mask)
    model.policy.optimizer.state.clear()


class MatchEnv(gym.Env):
    observation_space = ShapeEnv.observation_space
    action_space = ShapeEnv.action_space

    def __init__(self, backend, bank, history, seed, duration):
        self.match = StrategyMatch(backend, bank, duration)
        self.history = history
        self.models = [load(h["path"]) for h in history]
        self.rng = np.random.default_rng(seed)
        self.order = []
        self.records, self.resets = [], []
        self.sampled = Counter()
        self.changed = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if not self.order:
            self.order = self.rng.permutation(4).tolist()
        self.index = self.order.pop()
        self.key = KEYS[self.index]
        self.enemy = int(self.rng.integers(len(self.history)))
        match_seed = int(self.rng.integers(6200000, 6300000))
        self.match.reset(match_seed)
        self.resets.append(
            {
                "seed": match_seed,
                "learner": name(self.key),
                "opponent": self.enemy,
                "opponent_sha256": self.history[self.enemy]["sha256"],
            }
        )
        return self.match.observations()[self.index], {}

    def step(self, action):
        m = self.match
        before = m.result()
        old_events = len(m.option_events)
        team = self.key.team
        enemy = "yellow" if team == "blue" else "blue"

        def distance():
            return min(
                np.hypot(m.world.robot(k).x - m.world.ball.x, m.world.robot(k).y - m.world.ball.y)
                for k in KEYS
                if k.team == team
            )

        previous_distance = distance()
        obs = m.observations()
        models = [
            self.models[-1] if t == team else self.models[self.enemy] for t in ("blue", "yellow")
        ]
        actions = np.concatenate(
            [
                models[0].predict(obs[:2], deterministic=True)[0],
                models[1].predict(obs[2:], deterministic=True)[0],
            ]
        )
        self.changed += int(action[:10].argmax() != actions[self.index, :10].argmax())
        self.sampled[MODES[int(action[:10].argmax())]] += 1
        actions[self.index] = action
        m.step(actions)
        after = m.result()

        def delta(metric, t):
            return after[metric].get(t, 0) - before[metric].get(t, 0)

        reward = REWARD["goal_difference"] * (delta("score", team) - delta("score", enemy))
        reward += REWARD["carry_meter"] * delta("controlled_carry", team) + REWARD[
            "acquisition"
        ] * delta("acquisitions", team)
        for event in m.option_events[old_events:]:
            if event["type"] in (
                "receive_stop",
                "redirect_kick",
                "redirect_goal",
                "clearance_complete",
            ):
                reward += REWARD[event["type"]] * (1 if event["team"] == team else -1)
        for key, weight in [
            ("contact_events", "contact_event"),
            ("contact_checks", "contact_sample"),
            ("outside_checks", "outside_sample"),
        ]:
            reward -= REWARD[weight] * (after[key] - before[key])
        reward -= REWARD["safety_correction"] * delta("safety_correction", name(self.key))
        reward += (
            REWARD["possession_tick"] * (1 if m.current_owner.team == team else -1)
            if m.current_owner
            else REWARD["loose_progress_meter"] * (previous_distance - distance())
        )
        done = m.finished
        if done:
            self.records.append({**after, "learner": name(self.key), "opponent_index": self.enemy})
        else:
            m.restart_if_needed()
        return m.observations()[self.index], float(reward), done, False, {}


def train(backend, bank, out, initial, seed, rounds, steps, duration):
    out = Path(out)
    if out.exists():
        raise ValueError("Retain existing league attempts")
    out.mkdir(parents=True)
    initial = Path(initial)
    meta = json.loads(initial.with_suffix(".json").read_text())
    dataset_paths = []
    root = Path(__file__).resolve().parents[3]
    for receipt in meta["datasets"]:
        path = Path(receipt["path"])
        if not path.is_file() and "/runs/strategy-v1/" in str(path):
            path = (
                root
                / "policies/strategy_self_play_v1/preparation"
                / str(path).split("/runs/strategy-v1/", 1)[1]
            )
        assert digest(path) == receipt["sha256"], "Training data changed"
        dataset_paths.append(path)
    arrays = [np.load(p) for p in dataset_paths]
    x = np.concatenate([a["observations"] for a in arrays])
    y = np.concatenate([a["actions"] for a in arrays])
    groups = [np.flatnonzero(y[:, :10].argmax(1) == i) for i in range(10)]
    groups = [g for g in groups if len(g)]
    history = [
        {"path": str(initial), "sha256": digest(initial), "actor_sha256": actor_hash(load(initial))}
    ]
    ledger = []
    for generation in range(1, rounds + 1):
        stage = out / f"generation{generation}"
        stage.mkdir()
        env = MatchEnv(backend, bank, history, seed + generation * 101, duration)
        model = SelfPlayPPO.load(
            history[-1]["path"], env=Monitor(env), device="cpu", learning_rate=1e-4
        )
        model.set_random_seed(seed + generation * 101)
        freeze_targets(model)
        model.prepare_anchor(x, seed + generation * 101, groups)
        # Keep the same per-choice trust region when intent heads add frozen outputs.
        raw_outputs = model.policy.action_net.out_features
        anchor_lr = 3e-6 * raw_outputs / 17
        model.anchor_optimizer = torch.optim.Adam(model.actor_parameters(), lr=anchor_lr)
        model.actor_mse_limit = 0.01 * 17 / raw_outputs
        model.actor_delta_limit = 0.8
        before = actor_hash(model)
        targets_before = target_hash(model)
        start = time.monotonic()

        class Progress(BaseCallback):
            def _on_step(self):
                if self.num_timesteps % 8192 == 0:
                    print(
                        "Strategy PPO",
                        seed,
                        generation,
                        self.num_timesteps,
                        round(time.monotonic() - start, 1),
                        flush=True,
                    )
                return True

        model.learn(total_timesteps=steps, reset_num_timesteps=True, callback=Progress())
        assert target_hash(model) == targets_before, "Frozen intent function changed"
        assert actor_hash(model) != before, "No actual strategic update"
        with torch.no_grad():
            probe = model.policy.get_distribution(
                model.anchor_inputs[model.anchor_probe]
            ).distribution.mean
            changed = int(
                (
                    probe[:, :10].argmax(1)
                    != model.anchor_targets[model.anchor_probe, :10].argmax(1)
                ).sum()
            )
        metadata = {
            "seed": seed,
            "generation": generation,
            "training_seed": seed + generation * 101,
            "steps": model.num_timesteps,
            "actor_before": before,
            "actor_after": actor_hash(model),
            "targets_before": targets_before,
            "targets_after": target_hash(model),
            "accepted_updates": model.anchor_updates - model.anchor_rejections,
            "attempted_updates": model.anchor_updates,
            "training_seconds": time.monotonic() - start,
            "reward": REWARD,
            "opponent_pool": history.copy(),
            "motors": bank.descriptors,
            "datasets": meta["datasets"],
            "role_sampling": "shuffled cycles of all four robot/color assignments",
            "training_seed_range": [6200000, 6299999],
            "sampled_choices": dict(env.sampled),
            "exploratory_choice_changes": env.changed,
            "deterministic_probe_choice_changes": changed,
            "probe_rows": len(model.anchor_probe),
            "frozen_targets_and_motor_policies": True,
            "actor_mse_limit": model.actor_mse_limit,
            "anchor_learning_rate": anchor_lr,
            "raw_actor_outputs": raw_outputs,
            "actor_delta_limit": 0.8,
        }
        save(model, stage / "policy.zip", metadata)
        write(stage / "training.json", metadata)
        write(stage / "resets.json", env.resets)
        write(stage / "rollouts.json", env.records)
        write(stage / "updates.json", model.anchor_log)
        history.append(
            {
                "path": str(stage / "policy.zip"),
                "sha256": digest(stage / "policy.zip"),
                "actor_sha256": actor_hash(model),
            }
        )
        ledger.append(metadata)
        write(out / "league.json", {"seed": seed, "history": history, "updates": ledger})
        development = evaluate(backend, bank, [model, model], 8, 6300000, duration, trace=False)
        write(stage / "development.json", development)
        print(
            "Strategy generation finished",
            seed,
            generation,
            summarize(development["matches"]),
            flush=True,
        )
