"""Teachers and native curricula for continuous opponent policies (training only)."""

from dataclasses import asdict
import json
import math
from pathlib import Path
import gymnasium as gym
import numpy as np
import torch
from stable_baselines3.common.monitor import Monitor
from .dribble_refinement import AnchoredPPO
from .learned_opponents import CONTRACT, OBSERVATIONS, OpponentController, opponent_observation
from .model import Ball, Config, Referee, Robot, World, possession, wrap
from .env import goal_crossed
from .micro_training import Actor, collect, evaluate


def teacher_action(w, c):
    """Analytic labels; never imported by the deployed controller."""
    r, b, cfg, sign = w.robot(c.key), w.ball, c.cfg, w.attack_sign
    anchor = sign * (cfg.length / 2 - 0.18) if c.role == "keeper" else c.anchor_x
    if c.role == "keeper":
        goal = sign * cfg.length / 2
        y = b.y * sign * (goal - anchor) / max(0.01, sign * (goal - b.x))
        holder = next(
            (p for p in w.robots if p.key.team == "blue" and possession(w, p.key, cfg)), None
        )
        if c.style == "aim_tracking" and holder and sign * (goal - b.x) < 2.5:
            if sign * math.cos(holder.yaw) > 0.1:
                y = b.y + (anchor - b.x) * math.tan(holder.yaw)
        if sign * b.vx > 0.3:
            t = (anchor - b.x) / b.vx
            if 0 < t < 2:
                y = b.y + b.vy * t
        y = float(
            np.clip(
                y, -cfg.goal_width / 2 + cfg.robot_radius, cfg.goal_width / 2 - cfg.robot_radius
            )
        )
        vx, vy = 4 * (anchor - r.x), 8 * (y - r.y)
    else:
        y = c.anchor_y + 0.07 * math.sin(2 * math.pi * w.time / c.period + c.phase)
        if abs(b.vx) > 0.15:
            t = (anchor - b.x) / b.vx
            if 0 <= t <= 2.5:
                y = b.y + b.vy * t
        vx, vy = 2 * (anchor - r.x), 3 * (float(np.clip(y, -2.5, 2.5)) - r.y)
    scale = max(c.speed, math.hypot(vx, vy))
    angle = wrap(math.atan2(b.y - r.y, b.x - r.x) - r.yaw)
    return np.array(
        [sign * vx / scale, sign * vy / scale, np.clip(5 * angle / 4, -1, 1)], np.float32
    )


class ActionModel:
    observation_space = gym.spaces.Box(-np.inf, np.inf, (OBSERVATIONS,), dtype=np.float32)
    action_space = gym.spaces.Box(-1, 1, (3,), dtype=np.float32)
    action = np.zeros(3, np.float32)

    def predict(self, obs, deterministic=True):
        return self.action, None


class CollectingOpponent(OpponentController):
    def __init__(self, role, cfg, model=None, seed=127):
        super().__init__(role, cfg, model=model or ActionModel())
        self.behavior_model = model
        self.rng = np.random.default_rng(seed)
        self.x, self.y = [], []

    def action(self, w):
        obs, label = opponent_observation(w, self), teacher_action(w, self)
        self.x.append(obs.copy())
        self.y.append(label.copy())
        if self.behavior_model is not None and self.rng.random() >= 0.2:
            return self.behavior_model.predict(obs, deterministic=True)[0]
        return label


class OpponentChallengeEnv(gym.Env):
    """Physical interceptions: same control path, actual 1.8m/s passes / 3.5m/s shots."""

    metadata = {}

    def __init__(self, backend, cfg=Config(), role="keeper"):
        self.backend, self.cfg, self.role = backend, cfg, role
        self.action_space = ActionModel.action_space
        self.observation_space = ActionModel.observation_space
        self.controller = OpponentController(role, cfg, model=ActionModel())
        self.contract = CONTRACT
        self.task_version = "continuous_" + role + ".challenge.v1"

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        rng, c = self.np_random, self.controller
        c.configure(rng)
        sign = int(rng.choice([-1, 1]))
        c.anchor_x = (
            sign * (self.cfg.length / 2 - 0.18)
            if self.role == "keeper"
            else float(rng.uniform(-0.8, 0.8))
        )
        c.anchor_y = float(rng.uniform(-0.2, 0.2))
        c.phase, c.period = float(rng.uniform(-math.pi, math.pi)), float(rng.uniform(2, 4))
        if self.role == "defender":
            c.speed = float(rng.uniform(0.25, 0.55))
        target_y = (
            float(rng.uniform(-0.28, 0.28))
            if self.role == "keeper"
            else c.anchor_y + float(rng.uniform(-0.22, 0.22))
        )
        distance = float(rng.uniform(1.8, 2.8))
        x, y = c.anchor_x - sign * distance, float(rng.uniform(-0.4, 0.4))
        speed = 3.5 if self.role == "keeper" else 1.8
        t = distance / speed
        r = Robot(
            c.key,
            c.anchor_x,
            target_y + float(rng.uniform(-0.12, 0.12)),
            math.pi if sign == 1 else 0,
        )
        b = Ball(x, y, sign * speed, (target_y - y) / t)
        self.world = self.backend.reset(World(0, (r,), b, Referee.RUNNING, sign))
        self.steps = self.checks = self.contacts = 0
        self.finished = self.scored = False
        self.contact_time = None
        self.initial_speed = math.hypot(b.vx, b.vy)
        return self.observation(), self.info(False, "reset")

    def observation(self):
        return opponent_observation(self.world, self.controller)

    def step(self, action):
        if self.finished:
            raise RuntimeError("reset required")
        self.controller.model.action = action
        old = self.world
        self.world = self.backend.step({self.controller.key: self.controller.command(old)})
        self.steps += 1
        last = old.ball
        for w in getattr(self.backend, "last_substep_worlds", [self.world]):
            self.checks += 1
            self.scored |= goal_crossed(last, w.ball, self.cfg, w.attack_sign)
            last = w.ball
            r, b = w.robot(self.controller.key), w.ball
            contact = math.hypot(
                r.x - b.x, r.y - b.y
            ) <= self.cfg.robot_radius + self.cfg.ball_radius + 0.002 or possession(
                w, r.key, self.cfg
            )
            self.contacts += int(contact)
            if contact and self.contact_time is None:
                self.contact_time = w.time
        elapsed = self.steps * self.cfg.dt
        success = (
            self.contacts > 0 and not self.scored and (self.role == "defender" or elapsed >= 2.0)
        )
        missed = (
            self.role == "defender"
            and self.world.attack_sign * (self.world.ball.x - self.controller.anchor_x) > 0.25
            and not self.contacts
        )
        done = success or self.scored or missed
        truncated = elapsed >= 3.0 and not done
        self.finished = done or truncated
        reason = (
            "intercepted"
            if success
            else "goal_conceded"
            if self.scored
            else "missed"
            if missed
            else "timeout"
            if truncated
            else "running"
        )
        reward = 30 if success else -25 if done else -0.01
        return self.observation(), reward, done, truncated, self.info(success, reason)

    def info(self, success, reason):
        return {
            "is_success": bool(success),
            "reason": reason,
            "elapsed_seconds": self.steps * self.cfg.dt,
            "attack_sign": self.world.attack_sign,
            "physics_checks": self.checks,
            "ball_contact_checks": self.contacts,
            "goal_conceded": bool(self.scored),
            "initial_ball_speed": self.initial_speed,
            "role": self.role,
            "task": self.task_version,
        }

    def close(self):
        self.backend.close()


class ChallengeTeacher:
    def action(self, env):
        return teacher_action(env.world, env.controller)


def fit_opponent(model, x, y, epochs, seed):
    rng = np.random.default_rng(seed)
    inputs, labels = torch.as_tensor(x), torch.as_tensor(y)
    optimizer = torch.optim.Adam(model.actor_parameters(), lr=4e-4)
    groups = [
        np.flatnonzero(x[:, 21] > 0.5),
        np.flatnonzero(x[:, 21] < 0.5),
        np.flatnonzero(x[:, 23] > 0.5),
        np.flatnonzero(np.abs(y[:, 2]) > 0.2),
    ]
    groups = [g for g in groups if len(g)]
    for epoch in range(epochs):
        indices = np.concatenate([rng.choice(g, 12288, replace=True) for g in groups])
        rng.shuffle(indices)
        losses = []
        for batch in np.array_split(indices, len(indices) // 512):
            pred = model.policy.get_distribution(inputs[batch]).distribution.mean
            loss = ((pred - labels[batch]) ** 2).mean()
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            losses.append(loss.item())
        if (epoch + 1) % 10 == 0:
            print(f"Opponent clone {epoch + 1}/{epochs}: {np.mean(losses):.7f}", flush=True)


def train_opponents(backend, cfg, output, transfer, shooter, steps=32768, seed=127):
    from .learned_opponents import LearnedOpponentPlayEnv
    from .team import evaluate_team

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(1)
    torch.manual_seed(seed)
    controllers = {r: CollectingOpponent(r, cfg, seed=seed) for r in ("defender", "keeper")}
    env = LearnedOpponentPlayEnv(cfg, backend, controllers["defender"], controllers["keeper"])
    result = evaluate_team(env, transfer, 160, 1200000, shooter)
    (output / "teacher-full-play.json").write_text(json.dumps(result, indent=2) + "\n")
    models = {}
    datasets = {}
    for role, controller in controllers.items():
        challenge = OpponentChallengeEnv(backend, cfg, role)
        base = 1210000 if role == "defender" else 1220000
        x, y, records = collect(challenge, ChallengeTeacher(), 300, base)
        x, y = np.concatenate([controller.x, x]), np.concatenate([controller.y, y])
        datasets[role] = (x, y)
        model = AnchoredPPO(
            "MlpPolicy",
            Monitor(challenge),
            seed=seed,
            device="cpu",
            n_steps=1024,
            batch_size=256,
            n_epochs=5,
            learning_rate=3e-7,
            gamma=0.995,
            ent_coef=0,
            policy_kwargs={"net_arch": {"pi": [192, 192], "vf": [192, 192]}, "log_std_init": -4},
        )
        models[role] = model
        target = output / role
        target.mkdir(exist_ok=True)
        (target / "teacher-challenge.json").write_text(json.dumps(records, indent=2) + "\n")
        metadata = {
            "contract": CONTRACT,
            "config": asdict(cfg),
            "role": role,
            "seed": seed,
            "method": "native Webots behavioral cloning + DAgger + anchored PPO",
            "full_play_training_seeds": [1200000, 1200159],
            "challenge_training_seeds": [base, base + 299],
            "full_play_dagger_seeds": [1230000, 1230079],
            "challenge_dagger_seeds": [base + 2000, base + 2149],
            "development_seeds": [base + 1000, base + 1059],
            "physical_shot_speed": 3.5,
            "no_analytic_inference_fallback": True,
            "hardware_validated": False,
        }
        fit_opponent(model, x, y, 100, seed)
        save_opponent(model, target, "bc", metadata)
        report_opponent(challenge, model, target, "bc", base + 1000)
    controllers = {r: CollectingOpponent(r, cfg, models[r], seed + 1) for r in models}
    env = LearnedOpponentPlayEnv(cfg, backend, controllers["defender"], controllers["keeper"])
    result = evaluate_team(env, transfer, 80, 1230000, shooter)
    (output / "dagger-full-play.json").write_text(json.dumps(result, indent=2) + "\n")
    for role, model in models.items():
        challenge = OpponentChallengeEnv(backend, cfg, role)
        target = output / role
        base = 1210000 if role == "defender" else 1220000
        dx, dy, records = collect(challenge, ChallengeTeacher(), 150, base + 2000, model)
        x, y = datasets[role]
        x, y = (
            np.concatenate([x, controllers[role].x, dx]),
            np.concatenate([y, controllers[role].y, dy]),
        )
        (target / "dagger-challenge.json").write_text(json.dumps(records, indent=2) + "\n")
        np.savez_compressed(target / "demonstrations.npz", observations=x, actions=y)
        metadata = json.loads((target / "bc.json").read_text())
        fit_opponent(model, x, y, 80, seed + 1)
        save_opponent(model, target, "dagger", metadata)
        report_opponent(challenge, model, target, "dagger", base + 1000)
        results = {
            s: json.loads((target / (s + "-development.json")).read_text())
            for s in ("bc", "dagger")
        }
        best = max(
            results,
            key=lambda s: (results[s]["successes"], -(results[s]["mean_success_seconds"] or 100)),
        )
        model = AnchoredPPO.load(
            target / (best + ".zip"), env=Monitor(challenge), device="cpu", learning_rate=3e-7
        )
        model.set_random_seed(seed)
        model.prepare_anchor(x, seed, [np.arange(len(x))])
        for group in model.anchor_optimizer.param_groups:
            group["lr"] = 3e-6
        model.learn(total_timesteps=steps)
        metadata.update(
            anchor_reference=best,
            anchor_updates=model.anchor_updates,
            anchor_rejected_updates=model.anchor_rejections,
        )
        save_opponent(model, target, "policy", metadata)
        (target / "anchor-updates.json").write_text(json.dumps(model.anchor_log, indent=2) + "\n")
        report_opponent(challenge, model, target, "policy", base + 1000)


def save_opponent(model, target, stage, metadata):
    model.save(target / stage)
    (target / (stage + ".json")).write_text(
        json.dumps({**metadata, "stage": stage, "actual_steps": model.num_timesteps}, indent=2)
        + "\n"
    )


def report_opponent(env, model, target, stage, seed):
    result = evaluate(env, Actor(model), 60, seed)
    (target / (stage + "-development.json")).write_text(json.dumps(result, indent=2) + "\n")
    print(f"{env.role} {stage}: {result['successes']}/60", flush=True)
    return result
