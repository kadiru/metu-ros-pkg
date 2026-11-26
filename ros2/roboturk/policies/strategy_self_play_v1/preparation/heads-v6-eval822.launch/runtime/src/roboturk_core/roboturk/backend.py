"""Backend protocol and explicitly synthetic planar dynamics for fast contract tests."""

from dataclasses import replace
import math
from typing import Protocol

from .model import Command, Config, RobotKey, World, bounded, possession, world_vector, wrap


class Backend(Protocol):
    name: str

    def reset(self, world: World) -> World: ...
    def step(self, commands: dict[RobotKey, Command]) -> World: ...
    def close(self) -> None: ...


class ReferenceBackend:
    """Synthetic smoke-test backend. No wheel/contact/hardware fidelity is claimed."""

    name = "synthetic_planar"

    def __init__(self, cfg=Config()):
        self.cfg = cfg
        self.world = None
        self.kick_held = set()

    def reset(self, world):
        self.world = world
        self.kick_held.clear()
        return world

    def step(self, commands):
        if self.world is None:
            raise RuntimeError("reset must precede step")
        cfg, old = self.cfg, self.world
        robots = []
        ball = old.ball
        held = False
        next_kick_held = set()
        self.last_kick_events = []
        for robot in old.robots:
            c = bounded(commands.get(robot.key, Command()), cfg)
            vx, vy = world_vector(c.vx, c.vy, robot.yaw)
            moved = replace(
                robot,
                x=robot.x + vx * cfg.dt,
                y=robot.y + vy * cfg.dt,
                yaw=wrap(robot.yaw + c.omega * cfg.dt),
                vx=vx,
                vy=vy,
                omega=c.omega,
            )
            robots.append(moved)
            if c.kick > 0:
                next_kick_held.add(robot.key)
            if not held and possession(old, robot.key, cfg):
                if c.kick > 0 and robot.key not in self.kick_held:
                    self.last_kick_events.append((robot.key, old, c.kick))
                    ball = replace(
                        ball, vx=c.kick * math.cos(robot.yaw), vy=c.kick * math.sin(robot.yaw)
                    )
                    held = True
                elif c.dribbler > 0.5:
                    ball = replace(
                        ball,
                        x=moved.x + cfg.possession_x * math.cos(moved.yaw),
                        y=moved.y + cfg.possession_x * math.sin(moved.yaw),
                        vx=vx,
                        vy=vy,
                    )
                    held = True
        if not held or any(
            commands.get(r.key, Command()).kick > 0 and possession(old, r.key, cfg)
            for r in old.robots
        ):
            damping = math.exp(-0.35 * cfg.dt)
            ball = replace(
                ball,
                x=ball.x + ball.vx * cfg.dt,
                y=ball.y + ball.vy * cfg.dt,
                vx=ball.vx * damping,
                vy=ball.vy * damping,
            )
        self.kick_held = next_kick_held
        # Simple rigid-disc ball contact; intentionally not a substitute for Webots physics.
        if not held:
            for r in robots:
                dx, dy = ball.x - r.x, ball.y - r.y
                distance = math.hypot(dx, dy)
                minimum = cfg.robot_radius + cfg.ball_radius
                if 1e-9 < distance < minimum:
                    nx, ny = dx / distance, dy / distance
                    ball = replace(
                        ball,
                        x=r.x + nx * minimum,
                        y=r.y + ny * minimum,
                        vx=r.vx + nx * 0.1,
                        vy=r.vy + ny * 0.1,
                    )
        self.world = replace(old, time=old.time + cfg.dt, robots=tuple(robots), ball=ball)
        return self.world

    def close(self):
        pass
