"""Pass around a reactive midfield lane defender, then compose the learned scorer.

The opponent is a bounded analytic guard, not a learned/free-ranging adversary.
It stays on its initial longitudinal line, patrols the initial passing lane, and
tracks the predicted crossing of a rolling ball. It remains active after handoff.
"""

import math
import json
from pathlib import Path
import gymnasium as gym
import numpy as np
from .model import (
    skill_limits,
    compatible_policy_config,
    Ball,
    Command,
    Config,
    Intent,
    Referee,
    Robot,
    RobotKey,
    Skill,
    World,
    body_vector,
    bounded,
    possession,
    wrap,
    world_vector,
)
from .backend import ReferenceBackend
from .capture import CaptureTeacher
from .team import PassReceiveEnv, TransferState, TeamCheckpoint
from .team_contract import team_observation, team_action, team_commands, PASS_SPEED

DEFENDED_CONTRACT_V1 = "roboturk.defended_transfer.v1"
DEFENDED_CONTRACT_V2 = "roboturk.defended_transfer.v2"
DEFENDED_CONTRACT = "roboturk.defended_transfer.v3"
DEFENDER = RobotKey("yellow", 0)


def lane_clearance(world, receiver):
    b, r, d = world.ball, world.robot(receiver), world.robot(DEFENDER)
    vx, vy = r.x - b.x, r.y - b.y
    t = max(0.0, min(1.0, ((d.x - b.x) * vx + (d.y - b.y) * vy) / max(1e-9, vx * vx + vy * vy)))
    return math.hypot(d.x - b.x - t * vx, d.y - b.y - t * vy)


def defended_observation(world, passer, receiver, state, cfg, contract=DEFENDED_CONTRACT):
    base = team_observation(world, passer, receiver, state, cfg)
    d, b, r = world.robot(DEFENDER), world.ball, world.robot(receiver)
    extra = []
    for key in (passer, receiver):
        own = world.robot(key)
        extra.extend(v / 3 for v in body_vector(d.x - own.x, d.y - own.y, own.yaw))
        extra.extend(v / skill_limits(cfg).max_speed for v in body_vector(d.vx, d.vy, own.yaw))
    sign = world.direction(passer.team)
    extra.extend(
        [
            sign * (d.x - b.x) / 3,
            sign * (d.y - b.y) / 3,
            sign * (r.x - b.x) / 3,
            sign * (r.y - b.y) / 3,
        ]
    )
    extra.extend(body_vector(0, sign, r.yaw))
    extra.extend([lane_clearance(world, receiver), float(state.passer_dwell >= 20)])
    result = np.concatenate([base, extra]).astype(np.float32)
    if not np.all(np.isfinite(result)):
        raise ValueError("Nonfinite defended observation")
    result = augment_field_velocities(result)
    if contract == DEFENDED_CONTRACT_V2:
        return result
    if state.receiver_target is None:
        raise ValueError("The tactical play must assign a receiving point before inference")
    tx, ty = state.receiver_target
    return np.concatenate(
        [
            result,
            [
                sign * (tx - r.x) / 3,
                sign * (ty - r.y) / 3,
                math.hypot(tx - r.x, ty - r.y) / 3,
                sign * b.y / 3,
                sign * r.y / 3,
                math.hypot(r.vx, r.vy) / skill_limits(cfg).max_speed,
            ],
        ]
    ).astype(np.float32)


def augment_field_velocities(x):
    """Coordinate features only; no route or motion command is calculated here."""
    x = np.asarray(x, dtype=np.float32)
    ax, ay = x[..., 48:49], x[..., 49:50]
    return np.concatenate(
        [
            x,
            ay * x[..., 21:22] - ax * x[..., 22:23],
            ax * x[..., 21:22] + ay * x[..., 22:23],
            ay * x[..., 23:24] - ax * x[..., 24:25],
            ax * x[..., 23:24] + ay * x[..., 24:25],
        ],
        axis=-1,
    ).astype(np.float32)


def upgrade_actions(x, y):
    """Convert recorded v1 receiver body actions into the v2 attack frame."""
    result = np.array(y, dtype=np.float32, copy=True)
    vx, vy = y[..., 4], y[..., 5]
    result[..., 4] = x[..., 49] * vx - x[..., 48] * vy
    result[..., 5] = x[..., 48] * vx + x[..., 49] * vy
    return result


def defended_commands(action, world, passer, receiver, state, cfg):
    commands = team_commands(action, passer, receiver, state.passed, cfg)
    c, r = commands[receiver], world.robot(receiver)
    sign = world.direction(receiver.team)
    vx, vy = body_vector(sign * c.vx, sign * c.vy, r.yaw)
    commands[receiver] = Command(vx, vy, c.omega, c.dribbler, c.kick)
    return commands


def defended_action(commands, world, passer, receiver, cfg):
    action = team_action(commands, passer, receiver, cfg)
    r = world.robot(receiver)
    vx, vy = world_vector(float(action[4]), float(action[5]), r.yaw)
    sign = world.direction(receiver.team)
    action[4:6] = [sign * vx, sign * vy]
    return action


def receiving_point(world, passer, receiver, cfg):
    """Play-layer tactic: opposite flank, with room for the receiver to turn.

    This selects a stationary target once per play; it generates no robot command.
    The checkpoint learns motion, stopping, pass timing and interception control.
    """
    b, r, d = world.ball, world.robot(receiver), world.robot(DEFENDER)
    side = 1.0 if r.y > d.y else -1.0
    margin = cfg.width / 2 - 0.65
    return (r.x, float(np.clip(b.y - side * 2.2, -margin, margin)))


def tactical_state(world, passer, receiver, cfg):
    return TransferState(receiver_target=receiving_point(world, passer, receiver, cfg))


class LaneDefender:
    def __init__(self, cfg=Config()):
        self.cfg = cfg
        self.anchor_x = self.anchor_y = self.phase = 0.0
        self.speed = 0.4
        self.period = 3.0

    def command(self, world):
        r, b = world.robot(DEFENDER), world.ball
        target_y = self.anchor_y + 0.07 * math.sin(
            2 * math.pi * world.time / self.period + self.phase
        )
        if abs(b.vx) > 0.15:
            crossing = (self.anchor_x - b.x) / b.vx
            if 0 <= crossing <= 2.5:
                target_y = b.y + b.vy * crossing
        target_y = float(np.clip(target_y, -2.5, 2.5))
        vx, vy = 2 * (self.anchor_x - r.x), 3 * (target_y - r.y)
        scale = max(1.0, math.hypot(vx, vy) / self.speed)
        bx, by = body_vector(vx / scale, vy / scale, r.yaw)
        angle = wrap(math.atan2(b.y - r.y, b.x - r.x) - r.yaw)
        return bounded(Command(bx, by, float(np.clip(5 * angle, -4, 4)), 1.0), self.cfg)


class DefendedBackend:
    """Inject the opponent every control frame, including all scoring frames."""

    def __init__(self, backend, cfg):
        self.base, self.cfg = backend, cfg
        self.defender = LaneDefender(cfg)
        self.name = backend.name + "+reactive_lane_defender_v1"
        self.reset_metrics()

    def __getattr__(self, name):
        return getattr(self.base, name)

    @property
    def world(self):
        return self.base.world

    @world.setter
    def world(self, value):
        self.base.world = value

    def reset_metrics(self):
        self.failure = None
        self.ball_contacts = self.robot_contacts = self.checks = 0
        self.minimum_ball_distance = self.minimum_robot_distance = 99.0
        self.distance_moved = 0.0
        self.pass_clearance = None

    def reset(self, world):
        self.reset_metrics()
        return self.base.reset(world)

    def step(self, commands):
        before = self.world.robot(DEFENDER)
        commands = {**commands, DEFENDER: self.defender.command(self.world)}
        result = self.base.step(commands)
        for key, w, _ in self.base.last_kick_events:
            if key.team == "blue" and self.pass_clearance is None:
                receiver = next(r.key for r in w.robots if r.key.team == "blue" and r.key != key)
                self.pass_clearance = lane_clearance(w, receiver)
        for w in getattr(self.base, "last_substep_worlds", [result]):
            d, b = w.robot(DEFENDER), w.ball
            self.distance_moved += math.hypot(d.x - before.x, d.y - before.y)
            before = d
            distance = math.hypot(d.x - b.x, d.y - b.y)
            self.minimum_ball_distance = min(self.minimum_ball_distance, distance)
            self.checks += 1
            if distance <= self.cfg.robot_radius + self.cfg.ball_radius + 0.002 or possession(
                w, DEFENDER, self.cfg
            ):
                self.ball_contacts += 1
                self.failure = "defender_interception"
            for r in w.robots:
                if r.key.team == "blue":
                    distance = math.hypot(d.x - r.x, d.y - r.y)
                    self.minimum_robot_distance = min(self.minimum_robot_distance, distance)
                    if distance < 2 * self.cfg.robot_radius + 0.01:
                        self.robot_contacts += 1
                        self.failure = "defender_collision"
        return result

    def evidence(self):
        return {
            "opponent": "reactive_lane_defender_v1",
            "defender_speed_limit": self.defender.speed,
            "defender_ball_contacts": self.ball_contacts,
            "defender_robot_contacts": self.robot_contacts,
            "defender_physics_checks": self.checks,
            "minimum_ball_defender_distance": self.minimum_ball_distance,
            "minimum_robot_defender_distance": self.minimum_robot_distance,
            "defender_distance_moved": self.distance_moved,
            "pass_lane_clearance": self.pass_clearance,
        }


class DefendedEnv(PassReceiveEnv):
    task_version = "defended_pass_receive.randomized.v1"
    episode_steps = 1000

    def __init__(self, cfg=Config(), backend=None, contract=DEFENDED_CONTRACT):
        self.contract = contract
        super().__init__(cfg, DefendedBackend(backend or ReferenceBackend(cfg), cfg))
        self.observation_space = gym.spaces.Box(
            -np.inf, np.inf, (56 if contract == DEFENDED_CONTRACT_V2 else 62,), dtype=np.float32
        )

    def reset(self, *, seed=None, options=None):
        gym.Env.reset(self, seed=seed)
        rng = self.np_random
        sign = int(rng.choice([-1, 1]))
        side = int(rng.choice([-1, 1]))
        self.passer, self.receiver = (
            (RobotKey("blue", 0), RobotKey("blue", 1))
            if rng.random() < 0.5
            else (RobotKey("blue", 1), RobotKey("blue", 0))
        )
        px, py = sign * float(rng.uniform(-2.0, -1.2)), float(rng.uniform(-0.55, 0.55))
        rx, ry = px + sign * float(rng.uniform(1.8, 2.3)), py + side * float(rng.uniform(0.3, 0.55))
        yaw, ryaw = float(rng.uniform(-math.pi, math.pi)), float(rng.uniform(-math.pi, math.pi))
        b = Ball(
            px + self.cfg.possession_x * math.cos(yaw), py + self.cfg.possession_x * math.sin(yaw)
        )
        fraction = float(rng.uniform(0.5, 0.6))
        dx, dy = b.x + fraction * (rx - b.x), b.y + fraction * (ry - b.y)
        guard = self.backend.defender
        guard.anchor_x, guard.anchor_y = dx, dy
        guard.speed = float(rng.uniform(0.25, 0.55))
        guard.phase = float(rng.uniform(-math.pi, math.pi))
        guard.period = float(rng.uniform(2.0, 4.0))
        dy += float(rng.uniform(-0.025, 0.025))
        initial = World(
            0.0,
            (
                Robot(self.passer, px, py, yaw),
                Robot(self.receiver, rx, ry, ryaw),
                Robot(DEFENDER, dx, dy, math.atan2(b.y - dy, b.x - dx)),
            ),
            b,
            Referee.RUNNING,
            sign,
        )
        self.world = self.backend.reset(initial)
        self.state = (
            tactical_state(self.world, self.passer, self.receiver, self.cfg)
            if self.contract == DEFENDED_CONTRACT
            else TransferState()
        )
        self.steps, self.finished = 0, False
        self.initial_receiver = (rx, ry)
        self.scenario = {
            "attack_sign": sign,
            "passer_id": self.passer.id,
            "receiver_id": self.receiver.id,
            "initial_separation": math.hypot(rx - px, ry - py),
            "initial_passer_heading": yaw,
            "initial_receiver_heading": ryaw,
            "initial_lane_clearance": lane_clearance(self.world, self.receiver),
            "initial_receiver_side": side,
            "defender_initial_fraction": fraction,
            "defender_patrol_period": guard.period,
            "defender_patrol_phase": guard.phase,
        }
        return self.observation(), self.info(False, "reset")

    def observation(self):
        return defended_observation(
            self.world, self.passer, self.receiver, self.state, self.cfg, self.contract
        )

    def decode_action(self, action):
        return defended_commands(
            action, self.world, self.passer, self.receiver, self.state, self.cfg
        )

    def encode_commands(self, commands):
        return defended_action(commands, self.world, self.passer, self.receiver, self.cfg)

    def info(self, success, reason):
        return {
            **super().info(success, reason),
            **self.backend.evidence(),
            "receiver_lateral_displacement": abs(
                self.world.robot(self.receiver).y - self.initial_receiver[1]
            ),
        }

    def step(self, action):
        obs, reward, done, truncated, info = super().step(action)
        error = self.opponent_failure()
        if error:
            self.finished = True
            return obs, -25.0, True, False, self.info(False, error)
        return obs, reward, done, truncated, info

    def opponent_failure(self):
        return self.backend.failure


class DefendedTeacher:
    """Demonstration-only flank creation and pass timing; no runtime fallback."""

    def __init__(self, flank_offset=2.2):
        self.flank_offset = float(flank_offset)

    def action(self, env):
        w, cfg, s = env.world, env.cfg, env.state
        p, r, b, d = w.robot(env.passer), w.robot(env.receiver), w.ball, w.robot(DEFENDER)
        angle = wrap(math.atan2(r.y - b.y, r.x - b.x) - p.yaw)
        rangle = wrap(math.atan2(b.y - r.y, b.x - r.x) - r.yaw)
        omega = float(np.clip(2 * angle, -0.8, 0.8))
        bx, by = body_vector(b.x - p.x, b.y - p.y, p.yaw)
        pc = bounded(
            Command(3 * (bx - cfg.possession_x), -cfg.possession_x * omega + 3 * by, omega, 1), cfg
        )
        side = 1.0 if r.y > d.y else -1.0
        target_y = b.y + side * self.flank_offset
        vy = float(np.clip(2 * (target_y - r.y), -1.1, 1.1))
        vx = 0.0
        if s.receiver_target is not None:
            tx, target_y = s.receiver_target
            vx, vy = 2 * (tx - r.x), 2 * (target_y - r.y)
            scale = max(1.0, math.hypot(vx, vy) / 1.1)
            vx, vy = vx / scale, vy / scale
        rvx, rvy = body_vector(vx, vy, r.yaw)
        rc = bounded(Command(rvx, rvy, 5 * rangle, 1), cfg)
        if s.passed:
            pc = Command()
            rc = Command(omega=float(np.clip(5 * rangle, -8, 8)), dribbler=1)
            if math.hypot(b.x - r.x, b.y - r.y) < 0.7:
                rc = CaptureTeacher().command(
                    w, env.receiver, Intent(Skill.RECEIVE, (r.x, r.y)), cfg
                )
        elif (
            s.passer_dwell >= 30
            and abs(angle) < 0.035
            and abs(rangle) < 0.06
            and (
                math.hypot(s.receiver_target[0] - r.x, target_y - r.y)
                if s.receiver_target
                else abs(target_y - r.y)
            )
            < 0.06
            and math.hypot(r.vx, r.vy) < 0.15
            and math.hypot(p.vx, p.vy) < 0.15
            and lane_clearance(w, env.receiver) > 0.45
        ):
            pc = Command(kick=PASS_SPEED)
        return env.encode_commands({env.passer: pc, env.receiver: rc})


class DefendedCheckpoint(TeamCheckpoint):
    def __init__(self, path, cfg=Config()):
        from stable_baselines3 import PPO

        metadata = json.loads(Path(path).with_suffix(".json").read_text())
        if metadata["contract"] not in (
            DEFENDED_CONTRACT,
            DEFENDED_CONTRACT_V2,
            DEFENDED_CONTRACT_V1,
        ) or not compatible_policy_config(metadata["config"], cfg):
            raise ValueError("Defended policy manifest mismatch")
        self.contract = metadata["contract"]
        self.model = PPO.load(path, device="cpu")
        expected = {
            DEFENDED_CONTRACT_V1: (52,),
            DEFENDED_CONTRACT_V2: (56,),
            DEFENDED_CONTRACT: (62,),
        }[self.contract]
        if self.model.observation_space.shape != expected or self.model.action_space.shape != (7,):
            raise ValueError("Defended checkpoint shape mismatch")

    def initial_state(self, world, passer, receiver, cfg):
        return (
            tactical_state(world, passer, receiver, cfg)
            if self.contract == DEFENDED_CONTRACT
            else TransferState()
        )

    def action(self, env):
        raw = self.predict(env.world, env.passer, env.receiver, env.state, env.cfg)
        commands = self.decode_action(raw, env.world, env.passer, env.receiver, env.state, env.cfg)
        return env.encode_commands(commands)

    def decode_action(self, action, world, passer, receiver, state, cfg):
        if self.contract == DEFENDED_CONTRACT_V1:
            return team_commands(action, passer, receiver, state.passed, cfg)
        return defended_commands(action, world, passer, receiver, state, cfg)

    def predict(self, world, passer, receiver, state, cfg):
        observation = defended_observation(
            world,
            passer,
            receiver,
            state,
            cfg,
            DEFENDED_CONTRACT if self.contract == DEFENDED_CONTRACT else DEFENDED_CONTRACT_V2,
        )
        if self.contract == DEFENDED_CONTRACT_V1:
            observation = observation[:52]
        return self.model.predict(observation, deterministic=True)[0]
