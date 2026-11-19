"""Lightweight full-play Gymnasium environment; neural training is optional."""

import math
import gymnasium as gym
import numpy as np
from .team import scoring_handoff
from .self_play import SHAPES, make_play, mobile_observation


class Carrier:
    def __init__(self, role):
        obs, acts = SHAPES[role]
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (obs,), dtype=np.float32)
        self.action_space = gym.spaces.Box(-1, 1, (acts,), dtype=np.float32)
        self.value = np.zeros(acts, np.float32)

    def predict(self, obs, deterministic=True):
        return self.value.copy(), None


def proximity_penalty(world, key, weight):
    """Training-only cost for close robot spacing; never changes an action."""
    robot = world.robot(key)
    nearest = min(
        math.hypot(robot.x - p.x, robot.y - p.y) for p in world.robots if p.key != key
    )
    return weight * float(np.clip((0.38 - nearest) / 0.19, 0, 1)) ** 2


class MatchEnv(gym.Env):
    """One learning role; other roles are immutable snapshots during an episode."""

    metadata = {}

    def __init__(self, backend, cfg, role, current, pool, seed=217, proximity_cost=0.0):
        self.backend, self.cfg, self.role, self.current, self.pool = (
            backend,
            cfg,
            role,
            current,
            pool,
        )
        self.selector = np.random.default_rng(seed)
        self.carrier = Carrier(role)
        self.observation_space, self.action_space = (
            self.carrier.observation_space,
            self.carrier.action_space,
        )
        self.records, self.prefix_failures = [], []
        self.proximity_cost = proximity_cost

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.selector = np.random.default_rng(seed)
        for attempt in range(64):
            self.episode_seed = int(self.selector.integers(2100000, 2200000))
            self.pool_index = int(self.selector.integers(len(self.pool)))
            models = dict(self.current)
            adversaries = (
                ("transfer", "finish")
                if self.role in ("defender", "keeper")
                else ("defender", "keeper")
            )
            for r in adversaries:
                models[r] = self.pool[self.pool_index][r]
            if self.role in ("defender", "keeper"):
                models[self.role] = self.carrier
            self.env, self.transfer, self.finish = make_play(self.backend, self.cfg, models)
            self.env.reset(seed=self.episode_seed)
            self.shot, self.transfer_info, self.done = None, None, False
            self.carrier.value.fill(0)
            if self.role == "finish":
                while self.shot is None and not self.done:
                    self._advance(None)
                if self.done:
                    self.prefix_failures.append(self.record)
                    continue
            return self.observation(), {}
        raise RuntimeError(
            "No reachable finishing state in64 training prefixes; all failures retained"
        )

    def observation(self):
        if self.role in ("defender", "keeper"):
            c = getattr(self.env.backend, self.role)
            return mobile_observation(self.env.world, c)
        if self.role == "transfer":
            return self.env.observation()
        return self.shot.observation()

    def _advance(self, action):
        if self.role in ("defender", "keeper") and action is not None:
            self.carrier.value = np.asarray(action, np.float32)
        if self.shot is None:
            a = (
                action
                if self.role == "transfer" and action is not None
                else self.transfer.action(self.env)
            )
            _, reward, done, trunc, info = self.env.step(a)
            if done or trunc:
                self.transfer_info = info.copy()
                if info["is_success"]:
                    self.shot = scoring_handoff(self.env)
                    done = trunc = False
        else:
            a = (
                action
                if self.role == "finish" and action is not None
                else self.finish.action(self.shot)
            )
            _, reward, done, trunc, info = self.shot.step(a)
            self.env.world = self.shot.world
            # Same cross-attacker collision test as deployed full-play evaluation.
            for w in getattr(self.env.backend, "last_substep_worlds", [self.env.world]):
                p, r = w.robot(self.env.passer), w.robot(self.env.receiver)
                if math.hypot(p.x - r.x, p.y - r.y) < 2 * self.cfg.robot_radius + 0.01:
                    done, info["is_success"], info["reason"] = True, False, "robot_collision"
        self.done = bool(done or trunc)
        if self.done:
            self.record = {
                "seed": self.episode_seed,
                "is_success": bool(info["is_success"] and self.shot is not None),
                "reason": info["reason"],
                "transfer": self.transfer_info or info.copy(),
                "shooting": info.copy() if self.shot is not None else None,
                "elapsed_seconds": self.env.steps * self.cfg.dt
                + (self.shot.steps * self.cfg.dt if self.shot else 0),
                "opponent_pool_index": self.pool_index,
            }
        return reward, info

    def step(self, action):
        if self.done:
            raise RuntimeError("Reset the terminated match")
        old = self.env.world
        reward, info = self._advance(action)
        if self.role == "transfer" and self.shot is not None:
            while not self.done:
                _, info = self._advance(None)
        if self.role in ("defender", "keeper"):
            c = getattr(self.env.backend, self.role)
            r0, r1, b0, b1 = (
                old.robot(c.key),
                self.env.world.robot(c.key),
                old.ball,
                self.env.world.ball,
            )
            reward = (
                0.2 * (math.hypot(r0.x - b0.x, r0.y - b0.y) - math.hypot(r1.x - b1.x, r1.y - b1.y))
                - 0.001
                - proximity_penalty(self.env.world, c.key, self.proximity_cost)
            )
        else:
            reward = float(np.clip(reward, -2, 2)) * 0.1
        collision = False
        if self.done:
            reason = self.record["reason"]
            defensive = reason in ("keeper_save", "defender_interception")
            last = self.record["shooting"] or self.record["transfer"]
            collision = bool(
                "collision" in reason
                or last.get("defender_robot_contacts", 0)
                or last.get("keeper_robot_contacts", 0)
            )
            reward = (
                -40.0
                if collision
                else (
                    (25 if defensive else -25 if reason == "goal" else -5)
                    if self.role in ("defender", "keeper")
                    else (35 if reason == "goal" else -25)
                )
            )
            self.records.append(self.record)
        return (
            self.observation(),
            reward,
            self.done,
            False,
            {
                "is_success": bool(
                    self.done
                    and not collision
                    and (
                        info["reason"] in ("keeper_save", "defender_interception")
                        if self.role in ("defender", "keeper")
                        else info["is_success"]
                    )
                ),
                "reason": info["reason"],
            },
        )

    def close(self):
        pass  # One Supervisor owns and closes the native backend.
