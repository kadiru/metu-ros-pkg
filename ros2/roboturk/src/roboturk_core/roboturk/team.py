"""Two-robot transfer curriculum and a pass → receive → score play.

Training and inference share observable play memory. This is centralized
coordination, not decentralized multi-agent RL. Physics is never reset on handoff.
"""

from dataclasses import dataclass, asdict
import math
import json
from pathlib import Path
import gymnasium as gym
import numpy as np
from .backend import ReferenceBackend
from .capture import CaptureTeacher
from .model import (
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
)
from .shoot import ShootEnv
from .shoot_contract import shoot_action
from .team_contract import (
    TEAM_CONTRACT,
    TEAM_CONTRACT_V1,
    PASS_SPEED,
    team_action,
    team_commands,
    team_observation,
)


@dataclass
class TransferState:
    receiver_target: tuple[float, float] | None = None
    passed: bool = False
    received: bool = False
    passer_dwell: int = 0
    receiver_dwell: int = 0
    pass_step: int | None = None
    receive_step: int | None = None
    pass_distance: float | None = None
    pass_x: float | None = None
    pass_y: float | None = None
    receive_distance: float | None = None
    receive_speed: float | None = None
    applied_passes: int = 0
    possession_checks: int = 0
    lost_checks: int = 0


class TransferTeacher:
    """Demonstration only; no invocation in learned checkpoint inference."""

    def action(self, env):
        w, cfg, state = env.world, env.cfg, env.state
        p, r, b = w.robot(env.passer), w.robot(env.receiver), w.ball
        angle = wrap(math.atan2(r.y - b.y, r.x - b.x) - p.yaw)
        receiver_angle = wrap(math.atan2(b.y - r.y, b.x - r.x) - r.yaw)
        omega = float(np.clip(2.0 * angle, -0.8, 0.8))
        bx, by = body_vector(b.x - p.x, b.y - p.y, p.yaw)
        pc = bounded(
            Command(3 * (bx - cfg.possession_x), -cfg.possession_x * omega + 3 * by, omega, 1.0),
            cfg,
        )
        if state.passed:
            pc = Command()
        elif (
            state.passer_dwell >= 20
            and abs(angle) < 0.025
            and abs(receiver_angle) < 0.05
            and math.hypot(p.vx, p.vy) < 0.15
        ):
            pc = Command(kick=PASS_SPEED)
        rc = Command(omega=5 * receiver_angle, dribbler=1.0)
        if state.passed and math.hypot(b.x - r.x, b.y - r.y) < 0.65:
            rc = CaptureTeacher().command(w, env.receiver, Intent(Skill.RECEIVE, (r.x, r.y)), cfg)
        return team_action(
            {env.passer: pc, env.receiver: bounded(rc, cfg)}, env.passer, env.receiver, cfg
        )


class TeamCheckpoint:
    def __init__(self, path, cfg=Config()):
        from stable_baselines3 import PPO

        metadata = json.loads(Path(path).with_suffix(".json").read_text())
        if metadata["contract"] not in (
            TEAM_CONTRACT,
            TEAM_CONTRACT_V1,
        ) or not compatible_policy_config(metadata["config"], cfg):
            raise ValueError("Team policy manifest does not match contract/config")
        self.contract = metadata["contract"]
        self.model = PPO.load(path, device="cpu")

    def initial_state(self, world, passer, receiver, cfg):
        return TransferState()

    def action(self, env):
        raw = self.predict(env.world, env.passer, env.receiver, env.state, env.cfg)
        commands = self.decode_action(raw, env.world, env.passer, env.receiver, env.state, env.cfg)
        return env.encode_commands(commands)

    def decode_action(self, action, world, passer, receiver, state, cfg):
        return team_commands(action, passer, receiver, state.passed, cfg)

    def predict(self, world, passer, receiver, state, cfg):
        obs = team_observation(world, passer, receiver, state, cfg)
        if self.contract == TEAM_CONTRACT_V1:
            obs = obs[:34]
        return self.model.predict(obs, deterministic=True)[0]


class PassReceiveEnv(gym.Env):
    metadata = {"render_modes": []}
    task_version = "pass_receive.randomized.v1"
    episode_steps = 800

    def __init__(self, cfg=Config(), backend=None):
        self.cfg, self.backend = cfg, backend or ReferenceBackend(cfg)
        self.passer, self.receiver = RobotKey("blue", 0), RobotKey("blue", 1)
        self.action_space = gym.spaces.Box(-1, 1, (7,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (36,), dtype=np.float32)
        self.finished = True

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        rng = self.np_random
        direction = int(rng.choice([-1, 1]))
        # Roles vary across robot IDs; both are teammates.
        self.passer, self.receiver = (
            (RobotKey("blue", 0), RobotKey("blue", 1))
            if rng.random() < 0.5
            else (RobotKey("blue", 1), RobotKey("blue", 0))
        )
        px, py = direction * float(rng.uniform(-2.0, -0.7)), float(rng.uniform(-1.1, 1.1))
        rx, ry = px + direction * float(rng.uniform(1.3, 2.4)), py + float(rng.uniform(-0.9, 0.9))
        yaw = float(rng.uniform(-math.pi, math.pi))
        ryaw = float(rng.uniform(-math.pi, math.pi))
        b = Ball(
            px + self.cfg.possession_x * math.cos(yaw), py + self.cfg.possession_x * math.sin(yaw)
        )
        initial = World(
            0.0,
            (Robot(self.passer, px, py, yaw), Robot(self.receiver, rx, ry, ryaw)),
            b,
            Referee.RUNNING,
            direction,
        )
        self.world = self.backend.reset(initial)
        self.state, self.steps, self.finished = TransferState(), 0, False
        self.initial_receiver = (rx, ry)
        self.scenario = {
            "attack_sign": direction,
            "passer_id": self.passer.id,
            "receiver_id": self.receiver.id,
            "initial_separation": math.hypot(rx - px, ry - py),
            "initial_passer_heading": yaw,
            "initial_receiver_heading": ryaw,
        }
        return self.observation(), self.info(False, "reset")

    def observation(self):
        return team_observation(self.world, self.passer, self.receiver, self.state, self.cfg)

    def info(self, success, reason):
        p, r, b = self.world.robot(self.passer), self.world.robot(self.receiver), self.world.ball
        return {
            "pass_heading_error": wrap(math.atan2(r.y - b.y, r.x - b.x) - p.yaw),
            "receive_heading_error": wrap(math.atan2(b.y - r.y, b.x - r.x) - r.yaw),
            "passer_speed": math.hypot(p.vx, p.vy),
            "receiver_speed": math.hypot(r.vx, r.vy),
            "is_success": bool(success),
            "reason": reason,
            "steps": self.steps,
            "elapsed_seconds": round(self.steps * self.cfg.dt, 3),
            "backend": self.backend.name,
            "task": self.task_version,
            **self.scenario,
            **asdict(self.state),
        }

    def decode_action(self, action):
        return team_commands(action, self.passer, self.receiver, self.state.passed, self.cfg)

    def encode_commands(self, commands):
        return team_action(commands, self.passer, self.receiver, self.cfg)

    def step(self, action):
        if self.finished:
            raise RuntimeError("reset required")
        cfg, s, previous = self.cfg, self.state, self.world
        commands = self.decode_action(action)
        self.world = self.backend.step(commands)
        self.steps += 1
        samples = getattr(self.backend, "last_substep_worlds", [self.world])
        events = getattr(self.backend, "last_kick_events", [])
        error = None
        if commands[self.passer].kick and not s.passed:
            passes = [e for e in events if e[0] == self.passer]
            s.applied_passes += len(passes)
            if (
                s.passer_dwell < 20
                or len(passes) != 1
                or not possession(previous, self.passer, cfg)
            ):
                error = "invalid_pass"
            else:
                s.passed, s.pass_step = True, self.steps
                b, r = passes[0][1].ball, passes[0][1].robot(self.receiver)
                s.pass_x, s.pass_y = b.x, b.y
                s.pass_distance = math.hypot(r.x - b.x, r.y - b.y)
                if s.pass_distance < 1.0:
                    error = "pass_too_short"
        if not s.passed and not error:
            held = [possession(w, self.passer, cfg) for w in samples]
            s.possession_checks += len(held)
            s.lost_checks += sum(not h for h in held)
            if not all(held):
                error = "passer_lost_possession"
            r, b = self.world.robot(self.passer), self.world.ball
            stable = all(held) and math.hypot(r.vx - b.vx, r.vy - b.vy) < 0.2
            s.passer_dwell = s.passer_dwell + 1 if stable else 0
        if s.passed and not error:
            r, b = self.world.robot(self.receiver), self.world.ball
            stable = (
                all(possession(w, self.receiver, cfg) for w in samples)
                and math.hypot(r.vx - b.vx, r.vy - b.vy) < 0.2
            )
            s.receiver_dwell = s.receiver_dwell + 1 if stable else 0
            if s.receiver_dwell >= 20:
                s.received, s.receive_step = True, self.steps
                s.receive_distance = math.hypot(b.x - s.pass_x, b.y - s.pass_y)
                s.receive_speed = math.hypot(r.vx - b.vx, r.vy - b.vy)
                if s.receive_distance < 1.0:
                    error = "reception_too_short"
        for w in samples:
            p, r = w.robot(self.passer), w.robot(self.receiver)
            if math.hypot(p.x - r.x, p.y - r.y) < 2 * cfg.robot_radius + 0.01:
                error = "robot_collision"
        if (
            any(
                abs(r.x) > cfg.length / 2 - cfg.robot_radius
                or abs(r.y) > cfg.width / 2 - cfg.robot_radius
                for r in self.world.robots
            )
            or abs(self.world.ball.x) > cfg.length / 2
            or abs(self.world.ball.y) > cfg.width / 2
        ):
            error = "out"
        success = s.received and error is None
        terminated = success or error is not None
        truncated = self.steps >= self.episode_steps and not terminated
        self.finished = terminated or truncated
        p, r, b = self.world.robot(self.passer), self.world.robot(self.receiver), self.world.ball
        reward = -0.01 + (20 if success else -20 if error else 0)
        if s.passed:
            oldr, oldb = previous.robot(self.receiver), previous.ball
            reward += 2 * (
                math.hypot(oldr.x - oldb.x, oldr.y - oldb.y) - math.hypot(r.x - b.x, r.y - b.y)
            )
            reward += 0.05 if possession(self.world, self.receiver, cfg) else 0
        return (
            self.observation(),
            float(reward),
            bool(terminated),
            bool(truncated),
            self.info(
                success, error or ("received" if success else "timeout" if truncated else "running")
            ),
        )

    def close(self):
        self.backend.close()


def scoring_handoff(env):
    """Evaluator handoff with no physical reset, after measured stable reception."""
    if not env.state.received:
        raise ValueError("Cannot shoot before a completed reception")
    shot = getattr(env, "scoring_env_type", ShootEnv)(env.cfg, env.backend)
    shot.key, shot.world = env.receiver, env.world
    shot.intent = Intent(Skill.SHOOT, (env.world.attack_sign * env.cfg.length / 2, 0.0))
    shot.steps, shot.dwell, shot.finished = 0, 20, False
    shot.acquired, shot.capture_step = True, 0
    shot.capture_distance = shot.initial_target_distance = shot.target_distance(shot.world)
    shot.scenario = {"attack_sign": env.world.attack_sign, "moving": False}
    if hasattr(shot, "begin_finish"):
        shot.begin_finish()
    return shot


def evaluate_team(env, policy, episodes=200, seed=450000, shooter=None, recorder=None):
    records = []
    play = None
    if isinstance(policy, TeamCheckpoint) and shooter is not None:
        from .team_play import PassReceiveScorePlay

        play = PassReceiveScorePlay(policy, shooter, env.cfg)
    for i in range(episodes):
        env.reset(seed=seed + i)
        if not play and shooter is not None and hasattr(shooter, "reset"):
            shooter.reset()
        if play:
            play.reset(env.world, env.passer, env.receiver)
        if recorder:
            recorder.begin_episode(i, seed + i, env)
        while True:
            action = env.encode_commands(play.commands(env.world)) if play else policy.action(env)
            _, _, done, truncated, info = env.step(action)
            if play:
                play.observe(
                    getattr(env.backend, "last_substep_worlds", [env.world]),
                    env.backend.last_kick_events,
                )
                # Memory observes every applied kick; the task monitor deliberately
                # rejects premature/invalid kicks. Preserve those terminal failures
                # as outcomes rather than demanding a valid-state transition.
                if not (done or truncated) or info["is_success"]:
                    assert (
                        play.state.passed,
                        play.state.received,
                        play.state.passer_dwell,
                        play.state.receiver_dwell,
                    ) == (
                        env.state.passed,
                        env.state.received,
                        env.state.passer_dwell,
                        env.state.receiver_dwell,
                    ), f"Play/task state mismatch at seed {seed + i}, step {env.steps}"
            if recorder:
                recorder.step(env, info, None)
            if done or truncated:
                break
        transfer_info = info.copy()
        shot_info = None
        if info["is_success"] and shooter is not None:
            shot = scoring_handoff(env)
            while True:
                c = (
                    play.commands(shot.world)[env.receiver]
                    if play
                    else shooter.command(shot.world, shot.key, shot.intent, shot.cfg)
                )
                _, _, done, truncated, shot_info = shot.step(shoot_action(c, shot.cfg))
                env.world = shot.world
                if play:
                    play.observe(
                        getattr(env.backend, "last_substep_worlds", [env.world]),
                        env.backend.last_kick_events,
                    )
                # ShootEnv sends no passer command; backend defaults it to HOLD.
                if any(
                    math.hypot(
                        w.robot(env.passer).x - w.robot(env.receiver).x,
                        w.robot(env.passer).y - w.robot(env.receiver).y,
                    )
                    < 2 * env.cfg.robot_radius + 0.01
                    for w in getattr(env.backend, "last_substep_worlds", [env.world])
                ):
                    shot_info["is_success"], shot_info["reason"], done = (
                        False,
                        "robot_collision",
                        True,
                    )
                if hasattr(env, "opponent_failure"):
                    failure = env.opponent_failure()
                    shot_info.update(env.backend.evidence())
                    if failure:
                        shot_info["is_success"], shot_info["reason"], done = False, failure, True
                if recorder:
                    recorder.step(env, shot_info, shot)
                if done or truncated:
                    break
        records.append(
            {
                "seed": seed + i,
                "is_success": bool(
                    info["is_success"] and (shooter is None or shot_info["is_success"])
                ),
                "reason": shot_info["reason"] if shot_info else info["reason"],
                "elapsed_seconds": info["elapsed_seconds"]
                + (shot_info["elapsed_seconds"] if shot_info else 0),
                "transfer": transfer_info,
                "shooting": shot_info,
            }
        )
        if recorder:
            recorder.end_episode(env, records[-1])
        if (i + 1) % 20 == 0:
            print(
                f"Team evaluation {i + 1}/{episodes}: {sum(r['is_success'] for r in records)} successes",
                flush=True,
            )
    count = sum(r["is_success"] for r in records)
    p, z, n = count / episodes, 1.959963984540054, episodes
    center = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return {
        "episodes": records,
        "inference_play": "PassReceiveScorePlay" if play else "transfer_only_or_teacher_baseline",
        "successes": count,
        "success_rate": p,
        "wilson_95": [max(0.0, center - half), min(1.0, center + half)],
        "transfer_successes": sum(r["transfer"]["is_success"] for r in records),
        "mean_success_seconds": float(
            np.mean([r["elapsed_seconds"] for r in records if r["is_success"]])
        )
        if count
        else None,
        "mean_pass_distance": float(
            np.mean([r["transfer"]["pass_distance"] for r in records if r["transfer"]["passed"]])
        )
        if any(r["transfer"]["passed"] for r in records)
        else None,
        "possession_loss_episodes": sum(
            r["transfer"]["lost_checks"] > 0
            or bool(r["shooting"] and r["shooting"]["lost_possession_checks"])
            for r in records
        ),
    }
