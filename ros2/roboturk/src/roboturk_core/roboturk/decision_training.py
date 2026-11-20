"""Training-only decision teacher, cloning and native alternating self-play."""

from dataclasses import asdict
import json
import math
from pathlib import Path
from collections import Counter
import time
import gymnasium as gym
import numpy as np
import torch
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback
from .decision_rewards import boundary_cost, training_settings
from .decision_play import (
    ACTIONS,
    OBSERVATIONS,
    CONTRACT,
    Decision,
    DecisionEnv,
    encode,
    evaluate,
    load_manager,
    summarize,
)
from .self_play import (
    MobileFinishPlan,
    update_receiving_target,
    load_model,
    mobile_observation,
    digest,
)
from .self_play_training import SelfPlayPPO, write, actor_hash
from .self_play_match import Carrier, proximity_penalty
from .defended import tactical_state

TRAIN_ROLES = ("defender", "keeper", "manager")


class DecisionTeacher:
    """Only for demonstration labels/baselines; never imported by inference."""

    def reset(self, env):
        self.transfer = tactical_state(env.world, *env.keys, env.cfg)
        self.start_pass = env.world.attack_sign * env.world.ball.x < -0.5
        self.finishing = not self.start_pass
        self.owner = 0
        self.plan = None
        self.previous_target = None

    def action(self, env):
        w, cfg, p = env.world, env.cfg, env.play
        if self.start_pass and not self.finishing:
            self.transfer.passed = p.states[0].passed
            update_receiving_target(w, *env.keys, self.transfer, cfg)
            if p.dwell[1] >= 20 and p.kicks[0]:
                self.finishing, self.owner = True, 1
        if self.finishing:
            if self.plan is None:
                self.plan = MobileFinishPlan(w, env.keys[self.owner], cfg)
            self.plan.update(w, env.keys[self.owner], cfg)
            d = Decision(
                2 if self.plan.phase else 1,
                self.owner,
                self.transfer.receiver_target,
                self.plan.parking(w, env.keys[self.owner], cfg),
                self.plan.target,
            )
        else:
            d = Decision(
                0,
                0,
                self.transfer.receiver_target,
                (w.attack_sign * (cfg.length / 2 - 1.35), 0),
                (w.attack_sign * cfg.length / 2, (1 if w.ball.y >= 0 else -1) * 0.43),
            )
        d.receive_update = self.previous_target != d.receive
        self.previous_target = d.receive
        return encode(d, w, cfg)


class ShapeEnv(gym.Env):
    observation_space = gym.spaces.Box(-np.inf, np.inf, (OBSERVATIONS,), dtype=np.float32)
    action_space = gym.spaces.Box(-1, 1, (ACTIONS,), dtype=np.float32)


def new_manager(seed):
    return SelfPlayPPO(
        "MlpPolicy",
        ShapeEnv(),
        device="cpu",
        seed=seed,
        n_steps=1024,
        batch_size=256,
        n_epochs=5,
        learning_rate=1e-6,
        gamma=0.995,
        clip_range=0.1,
        ent_coef=0,
        policy_kwargs={"net_arch": [256, 256], "log_std_init": -4.5},
    )


def save(models, output, cfg, metadata):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    bundle = {}
    from .self_play import CONTRACTS

    for role, m in models.items():
        path = output / (role + ".zip")
        m.save(path)
        write(
            path.with_suffix(".json"),
            {
                **metadata,
                "contract": CONTRACT if role == "manager" else CONTRACTS[role],
                "role": role,
                "config": asdict(cfg),
                "actor_sha256": actor_hash(m),
                "validated_for_hardware": False,
            },
        )
        bundle[role] = {
            "path": str(path.resolve()),
            "sha256": digest(path),
            "actor_sha256": actor_hash(m),
        }
    write(output / "bundle.json", bundle)
    return bundle


def load(bundle, cfg):
    models = {}
    for r, item in bundle.items():
        if digest(item["path"]) != item["sha256"]:
            raise ValueError("Changed historical checkpoint " + r)
        models[r] = (
            load_manager(item["path"], cfg) if r == "manager" else load_model(item["path"], r, cfg)
        )
    return models


def collect(backend, cfg, models, count, seed, mix=0):
    env = DecisionEnv(backend, cfg, models)
    teacher = DecisionTeacher()
    rng = np.random.default_rng(seed)
    data = {r: ([], []) for r in TRAIN_ROLES}

    class Collector:
        def reset(self, env):
            teacher.reset(env)

        def action(self, env):
            obs = env.play.observation(env.world)
            label = teacher.action(env)
            data["manager"][0].append(obs.copy())
            data["manager"][1].append(label.copy())
            for r in ("defender", "keeper"):
                c = getattr(env, r)
                x = mobile_observation(env.world, c)
                data[r][0].append(x)
                data[r][1].append(c.action(env.world))
            return (
                models["manager"].predict(obs, deterministic=True)[0]
                if mix and rng.random() < mix
                else label
            )

    result = evaluate(env, count, seed, Collector())
    result["teacher_control"] = f"{1 - mix:.0%} teacher / {mix:.0%} learned manager per command"
    return {r: tuple(np.asarray(a, np.float32) for a in xy) for r, xy in data.items()}, result


def fit(model, x, y, epochs, seed):
    rng = np.random.default_rng(seed)
    inputs, targets = torch.as_tensor(x), torch.as_tensor(y)
    mode = np.argmax(y[:, :3], axis=1)
    groups = [np.flatnonzero(mode == i) for i in range(3)]
    # Reset and transition decisions are rare in long native trajectories. Sampling
    # them explicitly prevents a previous-action identity shortcut from dominating.
    previous = x[:, 46:56]
    reset = np.max(np.abs(previous), axis=1) < 1e-7
    switch = (
        (np.argmax(previous[:, :3], axis=1) != mode)
        | (np.argmax(previous[:, 3:5], axis=1) != np.argmax(y[:, 3:5], axis=1))
        | (np.sign(previous[:, 9]) != np.sign(y[:, 9]))
    )
    groups += [np.flatnonzero(reset), np.flatnonzero(switch)]
    if ACTIONS == 11:
        groups += [np.flatnonzero(y[:, 10] > 0)]
    groups = [g for g in groups if len(g)]
    optimizer = torch.optim.Adam(model.actor_parameters(), lr=3e-4)
    losses = []
    for epoch in range(epochs):
        ids = np.concatenate([rng.choice(g, 8192, replace=True) for g in groups])
        rng.shuffle(ids)
        for batch in np.array_split(ids, len(ids) // 512):
            pred = model.policy.get_distribution(inputs[batch]).distribution.mean
            # Targets need centimeter-scale accuracy; discrete logits use wide margins.
            weight = torch.tensor([1, 1, 1, 1, 1, 12, 12, 12, 12, 4, 2], dtype=torch.float32)
            loss = ((pred - targets[batch]) ** 2 * weight).mean()
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            losses.append(float(loss.detach()))
        if (epoch + 1) % 10 == 0:
            print(f"Decision cloning {epoch + 1}/{epochs}: {np.mean(losses[-48:]):.7f}", flush=True)
    return {
        "epochs": epochs,
        "seed": seed,
        "samples": len(x),
        "mode_counts": {str(i): int(sum(mode == i)) for i in range(3)},
        "last_mean_loss": float(np.mean(losses[-48:])),
        "reset_samples": int(reset.sum()),
        "switch_samples": int(switch.sum()),
    }


def bootstrap(backend, cfg, output, sources, seed=417):
    output = Path(output)
    models = {r: load_model(p, r, cfg) for r, p in sources.items()}
    models["manager"] = new_manager(seed)
    save(models, output / "initial", cfg, {"stage": "manager_random_initialization", "seed": seed})
    for stage, count, base, mix, epochs in [
        ("teacher", 300, 3000000, 0, 100),
        ("dagger", 150, 3010000, 0.8, 80),
    ]:
        data, result = collect(backend, cfg, models, count, base, mix)
        write(output / (stage + "-collection.json"), result)
        print(stage, summarize(result["episodes"]), flush=True)
        for role, (x, y) in data.items():
            path = output / (role + "-data.npz")
            if stage == "dagger":
                old = np.load(path)
                x = np.concatenate([old["observations"], x])
                y = np.concatenate([old["actions"], y])
            np.savez_compressed(path, observations=x, actions=y)
            if role == "manager":
                receipt = fit(models["manager"], x, y, epochs, seed)
                write(output / (stage + "-fit.json"), receipt)
        bundle = save(
            models,
            output / stage,
            cfg,
            {"stage": stage, "seed": seed, "sources": {r: digest(p) for r, p in sources.items()}},
        )
        development = evaluate(DecisionEnv(backend, cfg, models), 60, 3200000)
        write(output / (stage + "-development.json"), development)
        print(stage + " development", development["summary"], flush=True)
    write(output / "bootstrap-bundle.json", bundle)


class DecisionMatch(gym.Env):
    def __init__(self, backend, cfg, role, current, pool, seed):
        self.backend, self.cfg, self.role, self.current, self.pool = (
            backend,
            cfg,
            role,
            current,
            pool,
        )
        self.rng = np.random.default_rng(seed)
        self.carrier = Carrier(role) if role != "manager" else None
        self.observation_space = (
            ShapeEnv.observation_space if role == "manager" else self.carrier.observation_space
        )
        self.action_space = (
            ShapeEnv.action_space if role == "manager" else self.carrier.action_space
        )
        self.records = []
        self.resets = []
        self.sampled_choices = Counter()
        self.choice_disagreements = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.pool_index = int(self.rng.integers(len(self.pool)))
        self.episode_seed = int(self.rng.integers(3100000, 3200000))
        self.resets.append({"seed": self.episode_seed, "pool_index": self.pool_index})
        models = dict(self.current)
        for r in ("defender", "keeper") if self.role == "manager" else ("manager",):
            models[r] = self.pool[self.pool_index][r]
        if self.carrier is not None:
            self.carrier.value.fill(0)
            models[self.role] = self.carrier
        self.env = DecisionEnv(self.backend, self.cfg, models)
        self.env.reset(seed=self.episode_seed)
        return self.observation(), {}

    def observation(self):
        return (
            self.env.play.observation(self.env.world)
            if self.role == "manager"
            else mobile_observation(self.env.world, getattr(self.env, self.role))
        )

    def step(self, action):
        if self.role == "manager":
            baseline = self.current["manager"].predict(self.observation(), deterministic=True)[0]
            choice = (int(np.argmax(action[:3])), int(np.argmax(action[3:5])), bool(action[10] > 0))
            self.sampled_choices[str(choice)] += 1
            self.choice_disagreements += choice != (
                int(np.argmax(baseline[:3])),
                int(np.argmax(baseline[3:5])),
                bool(baseline[10] > 0),
            )
        if self.role != "manager":
            self.carrier.value = np.asarray(action, np.float32)
            action = self.env.play.manager.predict(
                self.env.play.observation(self.env.world), deterministic=True
            )[0]
        old = self.env.world
        _, reward, done, _, info = self.env.step(action)
        if self.role != "manager":
            c = getattr(self.env, self.role)
            a, b = old.robot(c.key), self.env.world.robot(c.key)
            reward = (
                0.2
                * (
                    math.hypot(a.x - old.ball.x, a.y - old.ball.y)
                    - math.hypot(b.x - self.env.world.ball.x, b.y - self.env.world.ball.y)
                )
                - 0.002
                - proximity_penalty(self.env.world, c.key, 0.3)
                - boundary_cost(self.env.world, c.key, self.cfg)
            )
            if done:
                reward = (
                    -40
                    if "collision" in info["reason"] or info["reason"] == "robot_field_exit"
                    else 25
                    if info["reason"] in ("keeper_save", "defender_interception")
                    else -25
                    if info["reason"] == "goal"
                    else -25
                )
        if done:
            info["opponent_pool_index"] = self.pool_index
            self.records.append(info)
        return (
            self.observation(),
            reward,
            done,
            False,
            {
                "is_success": bool(
                    done
                    and (
                        info["reason"] == "goal"
                        if self.role == "manager"
                        else info["reason"] in ("keeper_save", "defender_interception")
                    )
                ),
                "reason": info["reason"],
            },
        )

    def close(self):
        pass


def train(backend, cfg, output, rounds=3, steps=32768, seed=427):
    output = Path(output)
    league_path = output / "league.json"
    league = (
        json.loads(league_path.read_text())
        if league_path.exists()
        else {
            "snapshots": [json.loads((output / "bootstrap-bundle.json").read_text())],
            "updates": [],
        }
    )
    for generation in range(len(league["snapshots"]), rounds + 1):
        current = load(league["snapshots"][-1], cfg)
        history = [load(b, cfg) for b in league["snapshots"]]
        for role in TRAIN_ROLES:
            base = output / f"round{generation}" / role
            receipt = base / "training.json"
            if receipt.exists():
                saved = json.loads(receipt.read_text())
                if saved["actor_before"] == saved["actor_after"]:
                    raise ValueError("Failed unchanged-actor run needs explicit new experiment")
                b = json.loads((base / "bundle.json").read_text())
                current = load(b, cfg)
                if saved["steps"] != steps or saved["generation"] != generation:
                    raise ValueError("Resume parameters changed")
                if saved not in league["updates"]:
                    league["updates"].append(saved)
                continue
            before = save(
                current,
                base / "before",
                cfg,
                {"stage": "frozen_role_opponents", "generation": generation},
            )
            pool = history + ([load(before, cfg)] if role == "manager" else [])
            env = DecisionMatch(
                backend, cfg, role, current, pool, seed + generation * 31 + TRAIN_ROLES.index(role)
            )
            settings = training_settings(role)
            role_seed = seed + generation * 31 + TRAIN_ROLES.index(role)
            m = SelfPlayPPO.load(
                before[role]["path"],
                env=Monitor(env),
                device="cpu",
                learning_rate=settings["learning_rate"],
            )
            m.set_random_seed(role_seed)
            with torch.no_grad():
                if role == "manager":
                    m.policy.log_std.fill_(settings["target_log_std"])
                    m.policy.log_std[:5].fill_(settings["choice_log_std"])
                    m.policy.log_std[10].fill_(settings["choice_log_std"])
                else:
                    m.policy.log_std.fill_(settings["motion_log_std"])
            initial_log_std = m.policy.log_std.detach().tolist()
            data = np.load(output / (role + "-data.npz"))
            x = data["observations"]
            rng = np.random.default_rng(seed + generation)
            ids = rng.choice(len(x), min(50000, len(x)), replace=False)
            m.prepare_anchor(x[ids], seed + generation, groups=[np.arange(len(ids))])
            for group in m.anchor_optimizer.param_groups:
                group["lr"] = 2e-6
            m.actor_mse_limit = settings["mse_limit"]
            m.actor_delta_limit = settings["delta_limit"]
            old_hash = actor_hash(m)
            started = time.monotonic()

            class Progress(BaseCallback):
                def _on_step(self):
                    if self.num_timesteps % 8192 == 0:
                        print(
                            f"Decision self-play G{generation} {role}: {self.num_timesteps}/{steps}, {time.monotonic() - started:.1f}s",
                            flush=True,
                        )
                    return True

            m.learn(total_timesteps=steps, reset_num_timesteps=True, callback=Progress())
            current[role] = m
            after = save(
                current,
                base,
                cfg,
                {"stage": "self_play", "generation": generation, "learning_role": role},
            )
            row = {
                "generation": generation,
                "role": role,
                "steps": steps,
                "actor_before": old_hash,
                "actor_after": actor_hash(m),
                "accepted_updates": m.anchor_updates - m.anchor_rejections,
                "rejected_updates": m.anchor_rejections,
                "before": before,
                "after": after,
                "historical_pool": league["snapshots"].copy(),
                "complete_rollouts": len(env.records),
                "reset_count": len(env.resets),
                "seed": role_seed,
                "settings": settings,
                "initial_log_std": initial_log_std,
                "final_log_std": m.policy.log_std.detach().tolist(),
                "sampled_manager_choices": dict(env.sampled_choices),
                "choices_different_from_starting_manager": env.choice_disagreements,
                "seconds": time.monotonic() - started,
            }
            write(base / "rollouts.json", env.records)
            write(base / "resets.json", env.resets)
            write(base / "updates.json", m.anchor_log)
            write(receipt, row)
            if old_hash == row["actor_after"]:
                raise RuntimeError("No accepted actor change; retained failed run")
            league["updates"].append(row)
            print(f"Generation{generation} {role}: {row['accepted_updates']} accepted", flush=True)
        bundle = save(
            current,
            output / f"round{generation}" / "snapshot",
            cfg,
            {"stage": "snapshot", "generation": generation},
        )
        league["snapshots"].append(bundle)
        write(league_path, league)
        result = evaluate(DecisionEnv(backend, cfg, current), 60, 3200000)
        write(output / f"round{generation}" / "development.json", result)
        print("Generation", generation, result["summary"], flush=True)
