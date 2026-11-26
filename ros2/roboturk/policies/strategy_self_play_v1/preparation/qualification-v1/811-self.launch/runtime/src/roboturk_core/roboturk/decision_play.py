"""Learned tactical choices over frozen neural motion skills.

No scripted target planner or reception-triggered handoff is called here. Physical
memory is observed; the manager alone selects option, acting robot and targets.
"""

from dataclasses import asdict, dataclass
from collections import Counter
import json
import math
from pathlib import Path
import gymnasium as gym
import numpy as np
from .model import Ball, Command, Config, Referee, Robot, RobotKey, World, possession, wrap
from .defended import DEFENDER, defended_observation, lane_clearance
from .keeper import KEEPER
from .team import TransferState
from .env import goal_crossed
from .shoot_contract import shoot_command
from .self_play import (
    MobileOpponent,
    MobileDefendedBackend,
    MobileKeeperBackend,
    finishing_observation,
    relative_robot,
    mobile_transfer_commands,
)

CONTRACT = "roboturk.tactical_decision.v4"
OBSERVATIONS = 75
ACTIONS = 12
OPTIONS = ("pass", "dribble", "shoot")


@dataclass
class Decision:
    option: int
    owner: int
    receive: tuple[float, float]
    parking: tuple[float, float]
    aim: tuple[float, float]
    receive_update: bool = True


def decode(action, world, cfg):
    a = np.asarray(action, np.float32)
    if a.shape != (ACTIONS,) or not np.isfinite(a).all():
        raise ValueError("Expected twelve finite tactical outputs")
    a = np.clip(a, -1, 1)
    sx, sy, sign = cfg.length / 2 - 0.2, cfg.width / 2 - 0.2, world.attack_sign
    return Decision(
        int(np.argmax(a[:3])),
        int(np.argmax(a[3:5])),
        (sign * float(a[5]) * sx, sign * (1 if a[11] >= 0 else -1) * abs(float(a[6])) * sy),
        (sign * float(a[7]) * sx, sign * float(a[8]) * sy),
        (sign * cfg.length / 2, sign * (1 if a[9] >= 0 else -1) * (cfg.goal_width / 2 - 0.07)),
        bool(a[10] > 0),
    )


def encode(decision, world, cfg):
    d, sign = decision, world.attack_sign
    a = np.full(ACTIONS, -0.8, np.float32)
    a[d.option] = a[3 + d.owner] = 0.8
    a[5:9] = [
        sign * d.receive[0] / (cfg.length / 2 - 0.2),
        abs(d.receive[1]) / (cfg.width / 2 - 0.2),
        sign * d.parking[0] / (cfg.length / 2 - 0.2),
        sign * d.parking[1] / (cfg.width / 2 - 0.2),
    ]
    a[9] = sign * d.aim[1] / (cfg.goal_width / 2 - 0.06)
    a[10] = 0.8 if d.receive_update else -0.8
    a[11] = 0.8 if sign * d.receive[1] >= 0 else -0.8
    return a


class NeuralPlan:
    """Adapter only: coordinates and phase come directly from the manager."""

    def __init__(self, decision):
        self.target = decision.aim
        self.point = decision.parking
        self.phase = int(decision.option == 2)
        self.side = 1 if self.target[1] >= 0 else -1

    def parking(self, world, key, cfg):
        return self.point

    def update(self, world, key, cfg):
        pass


class DecisionPlay:
    """Deployable inference and measured memory; no episode monitor decisions."""

    def __init__(self, manager, transfer, finish, cfg=Config()):
        self.manager, self.transfer, self.finish, self.cfg = manager, transfer, finish, cfg

    def reset(self, world, keys):
        if len(keys) != 2 or keys[0] == keys[1] or any(k.team != "blue" for k in keys):
            raise ValueError("Two distinct blue robots required")
        self.keys = tuple(keys)
        self.initial = [(world.robot(k).x, world.robot(k).y) for k in keys]
        self.states = [TransferState(), TransferState()]
        self.dwell = [0, 0]
        self.kicks = [0, 0]
        self.last_kick_time = [-10.0, -10.0]
        self.carry = [0.0, 0.0]
        self.previous = np.zeros(ACTIONS, np.float32)
        self.last_time = world.time
        self.decision = decode(self.previous, world, self.cfg)
        self.calls = 0

    def observation(self, w):
        sign, cfg, b = w.attack_sign, self.cfg, w.ball
        values = []
        for key in (*self.keys, DEFENDER, KEEPER):
            r = w.robot(key)
            values.extend(
                [
                    sign * r.x / 4.5,
                    sign * r.y / 3,
                    sign * r.vx / 2,
                    sign * r.vy / 2,
                    sign * math.sin(r.yaw),
                    sign * math.cos(r.yaw),
                    r.omega / 8,
                    float(possession(w, key, cfg)),
                ]
            )
        values.extend([sign * b.x / 4.5, sign * b.y / 3, sign * b.vx / 4, sign * b.vy / 4])
        values.extend(min(d, 40) / 20 for d in self.dwell)
        values.extend(min(10, w.time - t) / 10 for t in self.last_kick_time)
        values.extend(min(3, k) / 3 for k in self.kicks)
        values.extend(v for x, y in self.initial for v in (sign * x / 4.5, sign * y / 3))
        values.extend(self.previous[:10])
        values.extend(float(s.passed) for s in self.states)
        values.extend(c / 4 for c in self.carry)
        values.append(min(w.time, 40) / 40)
        for key in self.keys:
            r = w.robot(key)
            from .model import body_vector

            values.extend(body_vector(b.x - r.x, b.y - r.y, r.yaw))
        values.extend(
            math.hypot(sign * cfg.length / 2 - w.robot(k).x, w.robot(k).y) / 4.5 for k in self.keys
        )
        values.extend(
            math.hypot(
                self.decision.parking[0] - w.robot(k).x, self.decision.parking[1] - w.robot(k).y
            )
            / 3
            for k in self.keys
        )
        values.extend(lane_clearance(w, k) / 3 for k in self.keys)
        values.extend(
            wrap(
                math.atan2(self.decision.aim[1] - b.y, self.decision.aim[0] - b.x) - w.robot(k).yaw
            )
            for k in self.keys
        )
        values.extend(math.hypot(w.robot(k).vx, w.robot(k).vy) / 2 for k in self.keys)
        result = np.asarray(values, np.float32)
        if result.shape != (OBSERVATIONS,) or not np.isfinite(result).all():
            raise ValueError(f"Invalid manager observation {result.shape}")
        return result

    def commands(self, world, action=None):
        if (
            world.referee != Referee.RUNNING
            or not world.ball.visible
            or not all(world.robot(k).visible for k in self.keys)
        ):
            return {k: Command() for k in self.keys}
        if action is None:
            action = self.manager.predict(self.observation(world), deterministic=True)[0]
        proposed = decode(action, world, self.cfg)
        if not proposed.receive_update:
            proposed.receive = self.decision.receive
        self.decision = proposed
        # Remember the executed option/robot/target, not uncertain raw score margins.
        # This is action encoding; it does not choose a behavior or perform a handoff.
        self.previous = encode(self.decision, world, self.cfg)
        # Keep the signed executed coordinate in the unchanged observation contract.
        self.previous[6] = world.attack_sign * self.decision.receive[1] / (self.cfg.width / 2 - 0.2)
        self.calls += 1
        d = self.decision
        key, other = self.keys[d.owner], self.keys[1 - d.owner]
        if d.option == 0:
            state = self.states[d.owner]
            state.receiver_target = d.receive
            state.passer_dwell, state.receiver_dwell = self.dwell[d.owner], self.dwell[1 - d.owner]
            obs = np.concatenate(
                [
                    defended_observation(world, key, other, state, self.cfg),
                    relative_robot(world, key, KEEPER, self.cfg),
                    relative_robot(world, other, KEEPER, self.cfg),
                ]
            ).astype(np.float32)
            a = self.transfer.predict(obs, deterministic=True)[0]
            return mobile_transfer_commands(a, world, key, other, state, self.cfg)
        plan = NeuralPlan(d)
        obs = finishing_observation(world, key, plan, self.cfg)
        a = np.asarray(self.finish.predict(obs, deterministic=True)[0]).copy()
        if d.option == 1:
            a[3] = -1  # Dribble has no kick actuator; manager decides when to shoot.
        return {key: shoot_command(a, self.cfg), other: Command()}

    def observe(self, previous, samples, kick_events):
        if not samples or not math.isclose(
            samples[-1].time - self.last_time, self.cfg.dt, abs_tol=1e-7
        ):
            raise ValueError("Exactly one measured control period required")
        w = samples[-1]
        for i, key in enumerate(self.keys):
            events = [e for e in kick_events if e[0] == key]
            if events:
                self.kicks[i] += len(events)
                self.last_kick_time[i] = w.time
                self.states[i].passed = True
            r, b = w.robot(key), w.ball
            held = all(possession(s, key, self.cfg) for s in samples)
            stable = held and math.hypot(r.vx - b.vx, r.vy - b.vy) < 0.2
            self.dwell[i] = self.dwell[i] + 1 if stable else 0
            self.states[1 - i].received = self.dwell[i] >= 20
            if held and possession(previous, key, self.cfg) and not events:
                self.carry[i] += max(0, w.attack_sign * (b.x - previous.ball.x))
        self.last_time = w.time


def load_manager(path, cfg=Config()):
    from stable_baselines3 import PPO

    meta = json.loads(Path(path).with_suffix(".json").read_text())
    if meta["contract"] != CONTRACT or meta["config"] != asdict(cfg):
        raise ValueError("Tactical manager manifest mismatch")
    m = PPO.load(path, device="cpu")
    if m.observation_space.shape != (OBSERVATIONS,) or m.action_space.shape != (ACTIONS,):
        raise ValueError("Tactical manager shape mismatch")
    return m


class DecisionEnv(gym.Env):
    """Independent physical event monitor; never selects tactics or targets."""

    def __init__(self, backend, cfg, models):
        self.cfg = cfg
        self.play = DecisionPlay(models.get("manager"), models["transfer"], models["finish"], cfg)
        self.defender = MobileOpponent("defender", cfg, model=models["defender"])
        self.keeper = MobileOpponent("keeper", cfg, model=models["keeper"])
        self.backend = MobileKeeperBackend(
            MobileDefendedBackend(backend, cfg, self.defender), cfg, self.keeper
        )
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (OBSERVATIONS,), dtype=np.float32)
        self.action_space = gym.spaces.Box(-1, 1, (ACTIONS,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        rng, cfg = self.np_random, self.cfg
        self.seed = int(seed) if seed is not None else int(rng.integers(3100000, 3200000))
        self.family = self.seed % 3
        sign, side = int(rng.choice([-1, 1])), int(rng.choice([-1, 1]))
        self.keys = (RobotKey("blue", int(rng.integers(2))),)
        self.keys += (RobotKey("blue", 1 - self.keys[0].id),)
        if self.family == 0:
            px, py = sign * float(rng.uniform(-2.0, -1.2)), float(rng.uniform(-0.55, 0.55))
            rx, ry = (
                px + sign * float(rng.uniform(1.8, 2.3)),
                py + side * float(rng.uniform(0.3, 0.55)),
            )
            yaw = float(rng.uniform(-math.pi, math.pi))
            dx, dy = px + 0.55 * (rx - px), py + 0.55 * (ry - py)
        elif self.family == 1:
            px, py = sign * float(rng.uniform(0.2, 1.1)), side * float(rng.uniform(1.5, 2.2))
            rx, ry = px - sign * 1.1, -side * 0.9
            yaw = float(rng.uniform(-math.pi, math.pi))
            dx, dy = px - sign * float(rng.uniform(0.7, 1.0)), py - side * 0.4
        else:
            px, py = sign * float(rng.uniform(3.02, 3.15)), float(rng.uniform(-0.06, 0.06))
            rx, ry = sign * 1.5, side * 1.8
            yaw = (0 if sign == 1 else math.pi) + float(rng.uniform(-0.25, 0.25))
            dx, dy = px - sign * 1.1, side * 0.7
        b = Ball(px + cfg.possession_x * math.cos(yaw), py + cfg.possession_x * math.sin(yaw))
        world = World(
            0.0,
            (
                Robot(self.keys[0], px, py, yaw),
                Robot(self.keys[1], rx, ry, float(rng.uniform(-math.pi, math.pi))),
                Robot(DEFENDER, dx, dy, math.atan2(b.y - dy, b.x - dx)),
            ),
            b,
            Referee.RUNNING,
            sign,
        )
        self.defender.configure(rng)
        self.defender.anchor_x, self.defender.anchor_y = dx, dy
        self.keeper.configure(rng)
        self.world = self.backend.reset(world)
        self.play.reset(self.world, self.keys)
        self.steps, self.finished, self.reason = 0, False, "running"
        self.events, self.trace = [], []
        self.decisions = Counter()
        self.receptions = set()
        self.reception_events = []
        self.controlled = [True, False]
        self.possession_checks = self.lost_checks = 0
        self.empty_shot_retriggers = 0
        self.goal_crossing_segment = None
        self.previous_fire = {k: False for k in self.keys}
        self.goal_crossing_y = None
        self.last_option = self.last_owner = None
        self.option_changes = self.owner_changes = 0
        return self.play.observation(self.world), self.info()

    def step(self, action):
        if self.finished:
            raise RuntimeError("Reset a terminated decision episode")
        old, cfg = self.world, self.cfg
        commands = self.play.commands(old, action)
        d = self.play.decision
        self.decisions[OPTIONS[d.option]] += 1
        self.option_changes += self.last_option is not None and self.last_option != d.option
        self.owner_changes += self.last_owner is not None and self.last_owner != d.owner
        if (
            self.steps % 10 == 0
            or self.last_option != d.option
            or self.last_owner != d.owner
            or d.receive_update
        ):
            self.trace.append(
                {
                    "step": self.steps,
                    "option": OPTIONS[d.option],
                    "owner": d.owner,
                    "receive": d.receive,
                    "parking": d.parking,
                    "aim": d.aim,
                    "receive_update": d.receive_update,
                }
            )
        self.last_option, self.last_owner = d.option, d.owner
        self.world = self.backend.step(commands)
        self.steps += 1
        samples, kicks = self.backend.last_substep_worlds, self.backend.last_kick_events
        self.play.observe(old, samples, kicks)
        error = None
        for key in self.keys:
            c = commands[key]
            edges = c.kick > 0 and not self.previous_fire[key]
            applied = [e for e in kicks if e[0] == key]
            if edges and (len(applied) != 1 or not possession(old, key, cfg)):
                completed_shot = (
                    not applied
                    and d.option == 2
                    and self.events
                    and self.events[-1]["type"] == "shoot"
                    and self.events[-1]["owner"] == self.keys.index(key)
                )
                if completed_shot:
                    # An empty retrigger after an applied shot has no physical effect.
                    # Keep counting it; do not turn a valid shot into a phantom second kick.
                    self.empty_shot_retriggers += 1
                else:
                    error = "invalid_kick"
            self.previous_fire[key] = c.kick > 0
            for _, w, speed in applied:
                owner = self.keys.index(key)
                self.controlled[owner] = False
                distance = math.hypot(w.attack_sign * cfg.length / 2 - w.ball.x, w.ball.y)
                self.events.append(
                    {
                        "type": OPTIONS[d.option],
                        "owner": owner,
                        "step": self.steps,
                        "time": w.time,
                        "ball": [w.ball.x, w.ball.y],
                        "speed": speed,
                        "goal_distance": distance,
                        "held": possession(old, key, cfg),
                        "decision_owner": d.owner,
                    }
                )
                if owner != d.owner or d.option == 1 or (d.option == 2 and distance > 2):
                    error = "invalid_kick"
            if self.controlled[self.keys.index(key)] and not applied:
                held = [possession(w, key, cfg) for w in samples]
                self.possession_checks += len(held)
                self.lost_checks += sum(not h for h in held)
                if not all(held):
                    error = "lost_possession"
        for i in range(2):
            if self.play.dwell[i] >= 20:
                self.controlled[i] = True
            if self.play.dwell[i] >= 20 and any(
                e["type"] == "pass" and e["owner"] != i for e in self.events
            ):
                if i not in self.receptions:
                    latest = next(
                        e for e in reversed(self.events) if e["type"] == "pass" and e["owner"] != i
                    )
                    self.reception_events.append(
                        {
                            "owner": i,
                            "step": self.steps,
                            "dwell": self.play.dwell[i],
                            "ball": [self.world.ball.x, self.world.ball.y],
                            "pass_displacement": math.hypot(
                                self.world.ball.x - latest["ball"][0],
                                self.world.ball.y - latest["ball"][1],
                            ),
                        }
                    )
                self.receptions.add(i)
        crossing = False
        last = old.ball
        for w in samples:
            if goal_crossed(last, w.ball, cfg, w.attack_sign):
                crossing = True
                line = w.attack_sign * (cfg.length / 2 + cfg.ball_radius)
                alpha = (line - last.x) / (w.ball.x - last.x)
                self.goal_crossing_y = last.y + alpha * (w.ball.y - last.y)
                self.goal_crossing_segment = [[last.x, last.y], [w.ball.x, w.ball.y]]
            last = w.ball
            if any(
                math.hypot(a.x - b.x, a.y - b.y) < 2 * cfg.robot_radius + 0.01
                for i, a in enumerate(w.robots)
                for b in w.robots[i + 1 :]
            ):
                error = "robot_collision"
            if any(
                abs(r.x) > cfg.length / 2 - cfg.robot_radius
                or abs(r.y) > cfg.width / 2 - cfg.robot_radius
                for r in w.robots
            ):
                error = error or "robot_field_exit"
        evidence = self.backend.evidence()
        collision = bool(
            evidence["defender_robot_contacts"]
            or evidence["keeper_robot_contacts"]
            or error == "robot_collision"
        )
        defense = self.backend.failure
        goal = crossing and bool(self.events) and self.events[-1]["type"] == "shoot"
        if collision:
            reason = "robot_collision"
        elif error and error != "lost_possession":
            reason = error
        elif defense:
            reason = defense
        elif goal:
            reason = "goal"
        elif error:
            reason = error
        elif (
            abs(self.world.ball.y) > cfg.width / 2
            or abs(self.world.ball.x) > cfg.length / 2 + cfg.ball_radius
        ):
            reason = "ball_out"
        elif self.steps >= 2000:
            reason = "timeout"
        else:
            reason = "running"
        self.reason, self.finished = reason, reason != "running"
        reward = 0.3 * old.attack_sign * (self.world.ball.x - old.ball.x) - 0.002
        if self.finished:
            reward = 35 if reason == "goal" else -40 if collision else -25
        return self.play.observation(self.world), reward, self.finished, False, self.info()

    def info(self):
        return {
            "seed": self.seed,
            "family": ("midfield", "advanced", "close")[self.family],
            "attack_sign": self.world.attack_sign,
            "initial_holder_id": self.keys[0].id,
            "steps": self.steps,
            "elapsed_seconds": round(self.steps * self.cfg.dt, 3),
            "reason": self.reason,
            "is_success": self.reason == "goal",
            "manager_calls": self.play.calls,
            "option_counts": dict(self.decisions),
            "option_changes": int(self.option_changes),
            "owner_changes": int(self.owner_changes),
            "controlled_carry": self.play.carry.copy(),
            "stable_receptions": len(self.receptions),
            "reception_events": self.reception_events.copy(),
            "possession_checks": self.possession_checks,
            "lost_possession_checks": self.lost_checks,
            "empty_shot_retriggers": self.empty_shot_retriggers,
            "events": self.events.copy(),
            "decisions": self.trace.copy(),
            "goal_crossing_y": self.goal_crossing_y,
            "goal_crossing_segment": self.goal_crossing_segment,
            "ball_final": [self.world.ball.x, self.world.ball.y],
            "robots_final": {f"{r.key.team}{r.key.id}": [r.x, r.y] for r in self.world.robots},
            **self.backend.evidence(),
        }

    def close(self):
        pass


def evaluate(env, count, seed, policy=None, recorder=None):
    records = []
    for i in range(count):
        env.reset(seed=seed + i)
        if policy is not None and hasattr(policy, "reset"):
            policy.reset(env)
        if recorder:
            recorder.begin_episode(i, env)
        while not env.finished:
            action = (
                policy.action(env)
                if policy is not None
                else env.play.manager.predict(env.play.observation(env.world), deterministic=True)[
                    0
                ]
            )
            env.step(action)
            if recorder:
                recorder.step(env)
        records.append(env.info())
        if recorder:
            recorder.end_episode(env)
        if (i + 1) % 10 == 0:
            print(f"Decision evaluation {i + 1}/{count}: {env.reason}", flush=True)
    return {
        "episodes": records,
        "summary": summarize(records),
        "manager_target_fallback": False,
        "scripted_handoff": False,
        "backend": env.backend.name,
    }


def summarize(records):
    counts = Counter(r["reason"] for r in records)
    stops = counts["defender_interception"] + counts["keeper_save"]
    return {
        "count": len(records),
        "goals": counts["goal"],
        "stops": stops,
        "clean": counts["goal"] + stops,
        "collisions": sum(v for k, v in counts.items() if "collision" in k),
        "reasons": dict(counts),
        "passes": sum(e["type"] == "pass" for r in records for e in r["events"]),
        "shots": sum(e["type"] == "shoot" for r in records for e in r["events"]),
        "receptions": sum(r["stable_receptions"] for r in records),
        "carry_episodes": sum(max(r["controlled_carry"]) >= 0.5 for r in records),
        "direct_goals": sum(
            r["reason"] == "goal" and not any(e["type"] == "pass" for e in r["events"])
            for r in records
        ),
    }
