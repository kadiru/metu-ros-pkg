"""Native alternating self-play with immutable historical opponents.

Analytic teachers only bootstrap the new spatial curriculum. After bootstrap,
PPO updates use rewards from complete plays against sampled frozen checkpoints.
"""

from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import time
import gymnasium as gym
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from .model import RobotKey, possession, wrap, Command, Intent, Skill, body_vector, bounded
from .defended import lane_clearance
from .capture import CaptureTeacher
from .team_contract import PASS_SPEED
from .keeper import FinishTeacher
from .team import evaluate_team
from .shoot_contract import shoot_action
from .dribble_refinement import AnchoredPPO
from .self_play_match import Carrier, MatchEnv
from .self_play import (
    CONTRACTS,
    ROLES,
    MobileOpponent,
    MobilePlayEnv,
    MobileFinishPlan,
    mobile_observation,
    finishing_observation,
    make_play,
    digest,
    outcomes,
)


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n")


def mobile_teacher(w, c):
    """Training-only mobile lane interception, pursuit and keeper advancement."""
    r, b, cfg, sign = w.robot(c.key), w.ball, c.cfg, w.attack_sign
    blue = [w.robot(RobotKey("blue", i)) for i in range(2)]
    holder = min(blue, key=lambda p: math.hypot(p.x - b.x, p.y - b.y))
    other = next(p for p in blue if p.key != holder.key)
    held = possession(w, holder.key, cfg)
    if c.role == "defender":
        if held and sign * b.x < 0.8:
            tx, ty = b.x + 0.6 * (other.x - b.x), b.y + 0.6 * (other.y - b.y)
        elif held:
            tx, ty = b.x + sign * 0.24, b.y
        else:
            dt = min(0.5, math.hypot(b.x - r.x, b.y - r.y) / max(0.1, c.speed + 1.8))
            tx, ty = b.x + b.vx * dt, b.y + b.vy * dt
    else:
        goal = cfg.length / 2
        depth = float(np.clip((3.2 - (goal - sign * b.x)) * 0.42, 0.18, 0.85))
        tx = sign * (goal - depth)
        ty = b.y * depth / max(0.3, goal - sign * b.x)
        if sign * b.vx > 0.3:
            dt = (tx - b.x) / b.vx
            if 0 < dt < 2:
                ty = b.y + b.vy * dt
        elif held and sign * b.x > 2.0 and sign * math.cos(holder.yaw) > 0.1:
            ty = b.y + (tx - b.x) * math.tan(holder.yaw)
        ty = float(np.clip(ty, -0.75, 0.75))
    tx = float(np.clip(tx, -cfg.length / 2 + 0.2, cfg.length / 2 - 0.2))
    ty = float(np.clip(ty, -cfg.width / 2 + 0.2, cfg.width / 2 - 0.2))
    vx, vy = 3 * (tx - r.x), 3 * (ty - r.y)
    # Training labels discourage ramming. Runtime contains no avoidance override.
    for p in w.robots:
        if p.key == c.key:
            continue
        dx, dy = r.x - p.x, r.y - p.y
        d = max(1e-6, math.hypot(dx, dy))
        if d < 0.38:
            push = 5 * (0.38 - d)
            vx += push * dx / d
            vy += push * dy / d
        if d < 0.5:
            # Relative closing-speed limit in demonstration labels only.
            radial = vx * dx / d + vy * dy / d
            minimum = p.vx * dx / d + p.vy * dy / d - 4 * max(0, d - 0.215)
            if radial < minimum:
                vx += (minimum - radial) * dx / d
                vy += (minimum - radial) * dy / d
    scale = max(c.speed, math.hypot(vx, vy))
    angle = wrap(math.atan2(b.y - r.y, b.x - r.x) - r.yaw)
    return np.array(
        [sign * vx / scale, sign * vy / scale, np.clip(5 * angle / 4, -1, 1)], np.float32
    )


class CollectOpponent(MobileOpponent):
    def __init__(self, role, cfg, data, model=None, mix=0, seed=217):
        super().__init__(role, cfg, model=model or Carrier(role))
        self.data, self.mix, self.rng = data, mix, np.random.default_rng(seed)

    def action(self, w):
        obs, label = mobile_observation(w, self), mobile_teacher(w, self)
        self.data[self.role][0].append(obs.copy())
        self.data[self.role][1].append(label.copy())
        return super().action(w) if self.rng.random() < self.mix else label


class MobileTransferTeacher:
    """Training-only anticipatory turn and faster off-ball feint."""

    def action(self, env):
        w, cfg, s = env.world, env.cfg, env.state
        p, r, b = w.robot(env.passer), w.robot(env.receiver), w.ball
        tx, ty = s.receiver_target
        angle = wrap(math.atan2(ty - b.y, tx - b.x) - p.yaw)
        live_angle = wrap(math.atan2(r.y - b.y, r.x - b.x) - p.yaw)
        rangle = wrap(math.atan2(b.y - r.y, b.x - r.x) - r.yaw)
        omega = float(np.clip(3 * angle, -1.4, 1.4))
        bx, by = body_vector(b.x - p.x, b.y - p.y, p.yaw)
        pc = bounded(
            Command(3 * (bx - cfg.possession_x), -cfg.possession_x * omega + 3 * by, omega, 1), cfg
        )
        vx, vy = 12 * (tx - r.x), 12 * (ty - r.y)
        scale = max(1, math.hypot(vx, vy) / 2.0)
        if math.hypot(tx - r.x, ty - r.y) < 0.045:
            vx = vy = 0
        rvx, rvy = body_vector(vx / scale, vy / scale, r.yaw)
        rc = bounded(Command(rvx, rvy, 5 * rangle, 1), cfg)
        if s.passed:
            pc, rc = Command(), Command(omega=float(np.clip(5 * rangle, -8, 8)), dribbler=1)
            if math.hypot(b.x - r.x, b.y - r.y) < 0.7:
                rc = CaptureTeacher().command(
                    w, env.receiver, Intent(Skill.RECEIVE, (r.x, r.y)), cfg
                )
        elif (
            s.passer_dwell >= 30
            and abs(live_angle) < 0.035
            and abs(rangle) < 0.06
            and math.hypot(tx - r.x, ty - r.y) < 0.06
            and math.hypot(r.vx, r.vy) < 0.15
            and math.hypot(p.vx, p.vy) < 0.15
            and lane_clearance(w, env.receiver) > 0.45
        ):
            pc = Command(kick=PASS_SPEED)
        return env.encode_commands({env.passer: pc, env.receiver: rc})


class CollectTransfer:
    def __init__(self, data, model=None, mix=0, seed=217):
        self.data, self.model, self.mix = data, model, mix
        self.rng, self.teacher = np.random.default_rng(seed), MobileTransferTeacher()

    def action(self, env):
        obs, label = env.observation(), self.teacher.action(env)
        self.data["transfer"][0].append(obs.copy())
        self.data["transfer"][1].append(label.copy())
        return (
            self.model.predict(obs, deterministic=True)[0]
            if self.rng.random() < self.mix
            else label
        )


class CollectFinish:
    def __init__(self, data, model=None, mix=0, seed=217):
        self.data, self.model, self.mix = data, model, mix
        self.rng, self.teacher = np.random.default_rng(seed), FinishTeacher()
        self.reset()

    def reset(self):
        self.plan = None

    def command(self, w, key, intent, cfg):
        from .shoot_contract import shoot_command

        if self.plan is None:
            self.plan = MobileFinishPlan(w, key, cfg)
        obs = finishing_observation(w, key, self.plan, cfg)
        label = shoot_action(self.teacher.planned_command(w, key, self.plan, cfg), cfg)
        self.data["finish"][0].append(obs.copy())
        self.data["finish"][1].append(label.copy())
        a = (
            self.model.predict(obs, deterministic=True)[0]
            if self.rng.random() < self.mix
            else label
        )
        return shoot_command(a, cfg)


def collect_full(backend, cfg, count, seed, models=None, mix=0):
    data = {r: ([], []) for r in ROLES}
    models = models or {r: None for r in ROLES}
    d = CollectOpponent("defender", cfg, data, models["defender"], mix, seed)
    k = CollectOpponent("keeper", cfg, data, models["keeper"], mix, seed + 1)
    env = MobilePlayEnv(cfg, backend, d, k)

    class Progress:
        def begin_episode(self, index, seed, env):
            self.index = index

        def step(self, env, info, shot):
            pass

        def end_episode(self, env, result):
            if (self.index + 1) % 5 == 0:
                print(
                    f"Collection {self.index + 1}/{count}: {result['reason']} in{result['elapsed_seconds']:.2f}s",
                    flush=True,
                )

    result = evaluate_team(
        env,
        CollectTransfer(data, models["transfer"], mix, seed + 2),
        count,
        seed,
        CollectFinish(data, models["finish"], mix, seed + 3),
        Progress(),
    )
    result["collection_control"] = (
        "analytic teachers" if not mix else f"per-command{mix:.0%} neural /{1 - mix:.0%} teacher"
    )
    return {
        r: tuple(np.asarray(a, np.float32) for a in values) for r, values in data.items()
    }, result


class SelfPlayPPO(AnchoredPPO):
    """PPO with a wider, measured actor trust region than previous micro tasks."""

    def train(self):
        previous = [p.detach().clone() for p in self.actor_parameters()]
        PPO.train(self)
        for _ in range(4):
            ids = self.anchor_rng.integers(0, len(self.anchor_inputs), 2048)
            pred = self.policy.get_distribution(self.anchor_inputs[ids]).distribution.mean
            loss = ((pred - self.anchor_targets[ids]) ** 2).mean()
            self.anchor_optimizer.zero_grad()
            loss.backward()
            self.anchor_optimizer.step()
        with torch.no_grad():
            d = (
                self.policy.get_distribution(
                    self.anchor_inputs[self.anchor_probe]
                ).distribution.mean
                - self.anchor_targets[self.anchor_probe]
            )
            mse, maximum = float((d**2).mean()), float(d.abs().max())
        reject = (
            not math.isfinite(mse)
            or mse > getattr(self, "actor_mse_limit", 3e-5)
            or maximum > getattr(self, "actor_delta_limit", 0.06)
        )
        if reject:
            with torch.no_grad():
                for p, old in zip(self.actor_parameters(), previous, strict=True):
                    p.copy_(old)
            self.anchor_optimizer.state.clear()
        self.anchor_updates += 1
        self.anchor_rejections += int(reject)
        self.anchor_log.append(
            {"steps": self.num_timesteps, "mse": mse, "max_delta": maximum, "rejected": reject}
        )


def new_model(role, source, seed):
    carrier = Carrier(role)
    source_model = PPO.load(source, device="cpu")

    class ShapeEnv(gym.Env):
        observation_space, action_space = carrier.observation_space, carrier.action_space

    model = SelfPlayPPO(
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
        policy_kwargs={"net_arch": source_model.policy.net_arch, "log_std_init": -4.0},
    )
    old = source_model.policy.state_dict()
    state = model.policy.state_dict()
    for name, v in state.items():
        if v.shape == old[name].shape:
            state[name] = old[name]
        elif name.endswith("policy_net.0.weight") or name.endswith("value_net.0.weight"):
            v.zero_()
            v[:, : old[name].shape[1]] = old[name]
        else:
            raise ValueError("Unsupported expansion: " + name)
    model.policy.load_state_dict(state)
    with torch.no_grad():
        model.policy.log_std.fill_(-4.0 if role in ("transfer", "finish") else -3.3)
    return model


def fit(model, x, y, role, epochs, seed):
    if len(x) == 0:
        raise ValueError("No bootstrap observations for " + role)
    rng = np.random.default_rng(seed)
    inputs, targets = torch.as_tensor(x), torch.as_tensor(y)
    groups = [np.arange(len(x))]
    if role == "finish":
        groups = [
            np.flatnonzero(x[:, 27] < 0.5),
            np.flatnonzero(x[:, 27] >= 0.5),
            np.flatnonzero(y[:, 3] > 0),
        ]
    elif role == "transfer":
        from .team_training import groups as transfer_groups

        groups = transfer_groups(x, y)
    groups = [g for g in groups if len(g)]
    opt = torch.optim.Adam(model.actor_parameters(), lr=3e-4)
    for epoch in range(epochs):
        ids = np.concatenate([rng.choice(g, 8192, replace=True) for g in groups])
        rng.shuffle(ids)
        losses = []
        for batch in np.array_split(ids, max(1, len(ids) // 512)):
            pred = model.policy.get_distribution(inputs[batch]).distribution.mean
            loss = ((pred - targets[batch]) ** 2).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
            losses.append(float(loss.detach()))
        if (epoch + 1) % 10 == 0:
            print(f"Bootstrap {role} {epoch + 1}/{epochs}: {np.mean(losses):.7f}", flush=True)


def actor_hash(model):
    h = hashlib.sha256()
    parameters = list(model.policy.mlp_extractor.policy_net.parameters()) + list(
        model.policy.action_net.parameters()
    )
    for p in parameters:
        h.update(p.detach().cpu().numpy().tobytes())
    return h.hexdigest()


def save_bundle(models, output, cfg, metadata):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    bundle = {}
    for role, model in models.items():
        path = output / (role + ".zip")
        model.save(path)
        write(
            path.with_suffix(".json"),
            {
                **metadata,
                "contract": CONTRACTS[role],
                "role": role,
                "config": asdict(cfg),
                "actor_sha256": actor_hash(model),
                "validated_for_hardware": False,
            },
        )
        bundle[role] = {
            "path": str(path),
            "sha256": digest(path),
            "actor_sha256": actor_hash(model),
        }
    write(output / "bundle.json", bundle)
    return bundle


def load_bundle(bundle, cfg):
    from .self_play import load_model

    result = {}
    for role in ROLES:
        if digest(bundle[role]["path"]) != bundle[role]["sha256"]:
            raise ValueError("Historical opponent checkpoint changed")
        result[role] = load_model(bundle[role]["path"], role, cfg)
    return result


def evaluate_bundle(backend, cfg, models, count, seed):
    env, transfer, finish = make_play(backend, cfg, models)
    return evaluate_team(env, transfer, count, seed, finish)


def bootstrap(backend, cfg, output, sources, seed=217):
    output = Path(output)
    torch.manual_seed(seed)
    models = {r: new_model(r, sources[r], seed + i) for i, r in enumerate(ROLES)}
    save_bundle(
        models, output / "warm-start", cfg, {"stage": "expanded_source_weights", "seed": seed}
    )
    for name, count, base, mix, epochs in [
        ("teacher", 200, 2000000, 0, 60),
        ("dagger", 100, 2010000, 0.8, 40),
    ]:
        data, result = collect_full(backend, cfg, count, base, models, mix)
        write(output / (name + "-collection.json"), result)
        print(name, outcomes(result), flush=True)
        for i, role in enumerate(ROLES):
            x, y = data[role]
            if name == "dagger":
                old = np.load(output / (role + "-data.npz"))
                x, y = np.concatenate([old["observations"], x]), np.concatenate([old["actions"], y])
            np.savez_compressed(output / (role + "-data.npz"), observations=x, actions=y)
            fit(models[role], x, y, role, epochs, seed + i)
        bundle = save_bundle(
            models,
            output / name,
            cfg,
            {"stage": name, "seed": seed, "sources": {r: digest(sources[r]) for r in ROLES}},
        )
        dev = evaluate_bundle(backend, cfg, models, 40, 2200000)
        write(output / (name + "-development.json"), dev)
        print(name + " development", outcomes(dev), flush=True)
    write(output / "bootstrap-bundle.json", bundle)


def train_league(backend, cfg, output, rounds=2, steps=32768, seed=227, prepared_bundle=None):
    output = Path(output)
    initial = json.loads((output / "bootstrap-bundle.json").read_text())
    completed = (
        json.loads((output / "league.json").read_text())
        if (output / "league.json").exists()
        else {"snapshots": [initial], "updates": []}
    )
    history = completed["snapshots"]
    progress = output / "training-progress.json"
    ledger = (
        json.loads(progress.read_text())["updates"] if progress.exists() else completed["updates"]
    )
    prepared_generation = len(history)
    for generation in range(len(history), rounds + 1):
        current = load_bundle(history[-1], cfg)
        prepared = None
        if prepared_bundle is not None and generation == prepared_generation:
            prepared = json.loads(Path(prepared_bundle).read_text())
            receipt = json.loads(Path(prepared_bundle).with_name("training.json").read_text())
            if receipt["parent"] != history[-1] or receipt["bundle"] != prepared:
                raise ValueError("Prepared actor provenance does not match the current league")
            if any(
                prepared[r]["actor_sha256"] != history[-1][r]["actor_sha256"]
                for r in ("transfer", "finish")
            ):
                raise ValueError("Spacing preparation unexpectedly changed an attacking actor")
            current = load_bundle(prepared, cfg)
        pool = [load_bundle(b, cfg) for b in history]
        # Defense first; attackers subsequently face this new defense and history.
        for role in ("defender", "keeper", "transfer", "finish"):
            target = output / f"round{generation}" / role
            records_path = output / f"round{generation}" / (role + "-rollouts.json")
            updates_path = output / f"round{generation}" / (role + "-updates.json")
            if target.with_suffix(".json").exists():
                from .self_play import load_model

                meta = json.loads(target.with_suffix(".json").read_text())
                saved = load_model(target.with_suffix(".zip"), role, cfg)
                if (
                    not records_path.exists()
                    or not updates_path.exists()
                    or actor_hash(saved) != meta["actor_after"]
                    or meta["generation"] != generation
                    or meta["ppo_steps"] != steps
                ):
                    raise ValueError(
                        "Incomplete or inconsistent saved role; retain and inspect before recovery"
                    )
                current[role] = saved
                if not any(x["generation"] == generation and x["role"] == role for x in ledger):
                    ledger.append(meta)
                print(f"Resumed completed round{generation} {role}", flush=True)
                continue
            before_dir = output / f"round{generation}" / ("before-" + role)
            if (before_dir / "bundle.json").exists():
                start_bundle = json.loads((before_dir / "bundle.json").read_text())
                frozen = load_bundle(start_bundle, cfg)
                if any(actor_hash(frozen[r]) != actor_hash(current[r]) for r in ROLES):
                    raise ValueError("Resume opponent actors differ from the saved snapshot")
            else:
                start_bundle = save_bundle(
                    current,
                    before_dir,
                    cfg,
                    {"stage": "frozen_training_opponents", "generation": generation},
                )
            training_pool = pool + (
                [load_bundle(start_bundle, cfg)] if role in ("transfer", "finish") else []
            )
            env = MatchEnv(
                backend,
                cfg,
                role,
                current,
                training_pool,
                seed + generation * 31 + ROLES.index(role),
                proximity_cost=0.3 if generation >= 3 and role in ("defender", "keeper") else 0,
            )
            defensive_revision = generation >= 3 and role in ("defender", "keeper")
            learning_rate = (
                3e-5 if defensive_revision else (3e-6 if role in ("defender", "keeper") else 8e-7)
            )
            path = start_bundle[role]["path"]
            model = SelfPlayPPO.load(
                path,
                env=Monitor(env),
                device="cpu",
                learning_rate=learning_rate,
            )
            model.actor_mse_limit = 3e-4 if defensive_revision else 3e-5
            model.actor_delta_limit = 0.15 if defensive_revision else 0.06
            model.set_random_seed(seed + generation * 31 + ROLES.index(role))
            data = np.load(output / (role + "-data.npz"))
            x = data["observations"]
            model.prepare_anchor(x, seed + generation, [np.arange(len(x))])
            for group in model.anchor_optimizer.param_groups:
                group["lr"] = 2e-6
            before = actor_hash(model)
            started = time.monotonic()

            class Progress(BaseCallback):
                def _on_step(self):
                    if self.num_timesteps % 8192 == 0:
                        print(
                            f"Self-play round{generation} {role}: {self.num_timesteps}/{steps}, {time.monotonic() - started:.1f}s",
                            flush=True,
                        )
                    return True

            model.learn(total_timesteps=steps, callback=Progress())
            current[role] = model
            target = output / f"round{generation}" / role
            model.save(target)
            metadata = {
                "contract": CONTRACTS[role],
                "role": role,
                "config": asdict(cfg),
                "generation": generation,
                "training_seed": seed + generation * 31 + ROLES.index(role),
                "ppo_steps": model.num_timesteps,
                "actor_before": before,
                "previous_generation_actor": history[-1][role]["actor_sha256"],
                "preparation": prepared,
                "actor_after": actor_hash(model),
                "updates": model.anchor_updates,
                "rejected_updates": model.anchor_rejections,
                "opponent_bundles": history
                + ([start_bundle] if role in ("transfer", "finish") else []),
                "training_only_seed_range": [2100000, 2199999],
                "method": "alternating PPO with historical opponents and actor trust region",
                "learning_rate": learning_rate,
                "proximity_cost": env.proximity_cost,
                "actor_mse_limit": model.actor_mse_limit,
                "actor_delta_limit": model.actor_delta_limit,
            }
            write(target.with_suffix(".json"), metadata)
            write(
                output / f"round{generation}" / (role + "-rollouts.json"),
                {"episodes": env.records, "failed_finish_prefixes": env.prefix_failures},
            )
            write(output / f"round{generation}" / (role + "-updates.json"), model.anchor_log)
            ledger.append(metadata)
            write(progress, {"snapshots": history, "updates": ledger})
            print(
                f"Self-play {role}: {model.anchor_updates - model.anchor_rejections}/{model.anchor_updates} accepted updates; actor changed {before != actor_hash(model)}",
                flush=True,
            )
        snapshot_dir = output / f"round{generation}" / "snapshot"
        if (snapshot_dir / "bundle.json").exists():
            bundle = json.loads((snapshot_dir / "bundle.json").read_text())
            frozen = load_bundle(bundle, cfg)
            if any(actor_hash(frozen[r]) != actor_hash(current[r]) for r in ROLES):
                raise ValueError("Completed snapshot differs on resume")
        else:
            bundle = save_bundle(
                current,
                snapshot_dir,
                cfg,
                {"stage": "self_play_snapshot", "generation": generation, "seed": seed},
            )
        history.append(bundle)
        dev = evaluate_bundle(backend, cfg, current, 60, 2200100)
        write(output / f"round{generation}" / "development.json", dev)
        print(f"Round{generation} development", outcomes(dev), flush=True)
        write(output / "league.json", {"snapshots": history, "updates": ledger})
    return history
