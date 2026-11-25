"""Backend protocol and explicitly synthetic planar dynamics for fast contract tests."""

from dataclasses import replace
import math
from typing import Protocol

from .carpet import SWITCH_RATIO, ground_step
from .model import (
    GRAVITY,
    ROBOT_HEIGHT,
    Command,
    Config,
    RobotKey,
    World,
    bounded,
    possession,
    drive_velocity,
    launch_velocity,
    wrap,
)


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
        self.switch_speed = SWITCH_RATIO * math.hypot(world.ball.vx, world.ball.vy)
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
            vx, vy = drive_velocity(robot, c, cfg.dt, cfg)
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
                    vx, vy, vz = launch_velocity(c.kick, robot.yaw, c.kick_elevation)
                    self.switch_speed = SWITCH_RATIO * math.hypot(vx, vy)
                    ball = replace(ball, vx=vx, vy=vy, vz=vz)
                    held = True
                elif c.dribbler > 0.5:
                    self.switch_speed = math.inf
                    ball = replace(
                        ball,
                        x=moved.x + cfg.possession_x * math.cos(moved.yaw),
                        y=moved.y + cfg.possession_x * math.sin(moved.yaw),
                        vx=vx,
                        vy=vy,
                    )
                    held = True
        if ball.z > cfg.ball_radius or ball.vz > 0:
            height = ball.z + ball.vz * cfg.dt - GRAVITY * cfg.dt**2 / 2
            ball = replace(
                ball,
                x=ball.x + ball.vx * cfg.dt,
                y=ball.y + ball.vy * cfg.dt,
                z=max(cfg.ball_radius, height),
                vz=ball.vz - GRAVITY * cfg.dt if height > cfg.ball_radius else 0.0,
            )
        elif not held or self.last_kick_events:
            speed = math.hypot(ball.vx, ball.vy)
            if speed:
                travel, end = ground_step(speed, self.switch_speed, cfg.dt)
                ux, uy = ball.vx / speed, ball.vy / speed
                ball = replace(
                    ball, x=ball.x + ux * travel, y=ball.y + uy * travel, vx=ux * end, vy=uy * end
                )
        self.kick_held = next_kick_held
        # Simple rigid-disc ball contact; intentionally not a substitute for Webots physics.
        if not held and ball.z <= ROBOT_HEIGHT + cfg.ball_radius:
            for r in robots:
                dx, dy = ball.x - r.x, ball.y - r.y
                distance = math.hypot(dx, dy)
                minimum = cfg.robot_radius + cfg.ball_radius
                if 1e-9 < distance < minimum:
                    self.switch_speed = math.inf
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
