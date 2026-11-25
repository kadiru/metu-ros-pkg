"""Webots ODE contacts with isotropic carpet impulses and an acceleration-limited drive.

This is a behavior-training prototype, NOT a calibrated omniwheel model.
No ball teleportation is used during step. Reset is the only pose reset.
"""

from dataclasses import replace
import math

from .carpet import carpet_velocity
from .model import (
    Ball,
    Command,
    Config,
    Robot,
    bounded,
    possession,
    drive_velocity,
    launch_velocity,
)


class WebotsBackend:
    name = "webots_ideal_body_velocity"

    def __init__(self, supervisor, cfg=Config()):
        defaults = Config()
        for field in ("length", "width", "goal_width", "robot_radius", "ball_radius", "ball_mass"):
            if getattr(cfg, field) != getattr(defaults, field):
                raise ValueError(f"Author matching world/PROTO geometry before changing {field}")
        self.supervisor, self.cfg = supervisor, cfg
        self.basic_ms = int(supervisor.getBasicTimeStep())
        self.substeps = round(cfg.dt * 1000 / self.basic_ms)
        if not math.isclose(self.substeps * self.basic_ms, cfg.dt * 1000):
            raise ValueError("Control period must be a multiple of Webots basicTimeStep")
        self.ball = supervisor.getFromDef("BALL")
        self.nodes = {}
        self.world = None
        self.kick_held = set()
        self.epoch = 0.0
        self.closed = False

    def reset(self, world):
        self.world = world
        self.kick_held.clear()
        for r in world.robots:
            name = f"{r.key.team.upper()}_{r.key.id}"
            node = self.supervisor.getFromDef(name)
            if node is None:
                # World has local EXTERNPROTO declarations; entities can be spawned on reset.
                color = "0.05 0.25 0.95" if r.key.team == "blue" else "0.95 0.8 0.05"
                self.supervisor.getRoot().getField("children").importMFNodeFromString(
                    -1, f'DEF {name} SslRobot {{ name "{name}" color {color} }}'
                )
                node = self.supervisor.getFromDef(name)
            self.nodes[r.key] = node
        # Remove previous episode entities absent from the new scenario.
        wanted = {r.key for r in world.robots}
        for key in list(self.nodes):
            if key not in wanted:
                self.nodes.pop(key).remove()
        for r in world.robots:
            node = self.nodes[r.key]
            node.getField("translation").setSFVec3f([r.x, r.y, 0.075])
            node.getField("rotation").setSFRotation([0.0, 0.0, 1.0, r.yaw])
            node.resetPhysics()
        self.ball.getField("translation").setSFVec3f(
            [world.ball.x, world.ball.y, max(world.ball.z, self.cfg.ball_radius)]
        )
        self.ball.resetPhysics()
        # Flush pending pose/physics resets before setting velocities: Webots otherwise
        # clears the initial ball velocity when it applies resetPhysics at the step boundary.
        if self.supervisor.step(self.basic_ms) == -1:
            raise RuntimeError("Webots ended during reset")
        for r in world.robots:
            self.nodes[r.key].setVelocity([r.vx, r.vy, 0.0, 0.0, 0.0, r.omega])
        self.ball.setVelocity([world.ball.vx, world.ball.vy, world.ball.vz, 0.0, 0.0, 0.0])
        if self.supervisor.step(0) == -1:
            raise RuntimeError("Webots ended during reset")
        self.epoch = self.supervisor.getTime() - world.time
        # Reset settling is outside episode time; return the simulator's actual state.
        self.world = self.read()
        return self.world

    def read(self):
        robots = []
        for r in self.world.robots:
            node = self.nodes[r.key]
            p, v, orientation = node.getPosition(), node.getVelocity(), node.getOrientation()
            robots.append(
                Robot(
                    r.key, p[0], p[1], math.atan2(orientation[3], orientation[0]), v[0], v[1], v[5]
                )
            )
        p, v = self.ball.getPosition(), self.ball.getVelocity()
        return replace(
            self.world,
            time=self.supervisor.getTime() - self.epoch,
            robots=tuple(robots),
            ball=Ball(p[0], p[1], v[0], v[1], z=p[2], vz=v[2]),
        )

    def advance_ball(self, launch):
        if launch is None:
            velocity = self.ball.getVelocity()
        else:
            velocity = [*launch, 0.0, 0.0, 0.0]
            self.ball.setVelocity(velocity)
        radius = self.cfg.ball_radius
        if self.ball.getPosition()[2] > radius + 0.0001 or abs(velocity[2]) >= 0.03:
            return
        dt = self.basic_ms / 1000
        target = carpet_velocity(velocity, dt, radius)
        mass = self.cfg.ball_mass
        inertia = 0.4 * mass * radius**2
        # Apply the carpet impulse over the step. ODE combines it with robot,
        # dribbler and goal contacts. A kick sets the actual pre-friction speed.
        if target[:2] != velocity[:2]:
            self.ball.addForce(
                [(target[i] - velocity[i]) * mass / dt for i in (0, 1)] + [0.0], False
            )
        if target[3:5] != velocity[3:5]:
            self.ball.addTorque(
                [(target[i] - velocity[i]) * inertia / dt for i in (3, 4)] + [0.0], False
            )

    def step(self, commands):
        if self.world is None:
            raise RuntimeError("reset must precede step")
        cfg = self.cfg
        commands = {k: bounded(c, cfg) for k, c in commands.items()}
        kicked = set()
        ball_launched = False
        self.last_substep_worlds = []
        self.last_kick_events = []
        for _ in range(self.substeps):
            current = self.read()
            launch = None
            for r in current.robots:
                node = self.nodes[r.key]
                c = commands.get(r.key, Command())
                vx, vy = drive_velocity(r, c, self.basic_ms / 1000, cfg)
                vertical = node.getVelocity()[2]
                node.setVelocity([vx, vy, vertical, 0.0, 0.0, c.omega])
                if not ball_launched and possession(current, r.key, cfg):
                    if c.kick > 0 and r.key not in self.kick_held and r.key not in kicked:
                        launch = launch_velocity(c.kick, r.yaw, c.kick_elevation)
                        ball_launched = c.kick_elevation > 0
                        kicked.add(r.key)
                        self.last_kick_events.append((r.key, current, c.kick))
                    elif c.dribbler > 0.5 and r.key not in kicked:
                        # Bounded spring/damper attraction at the mouth. Calibrate before transfer.
                        x = r.x + cfg.possession_x * math.cos(r.yaw)
                        y = r.y + cfg.possession_x * math.sin(r.yaw)
                        fx = 8.0 * (x - current.ball.x) + 0.15 * (vx - current.ball.vx)
                        fy = 8.0 * (y - current.ball.y) + 0.15 * (vy - current.ball.vy)
                        norm = max(1.0, math.hypot(fx, fy) / 0.4)
                        self.ball.addForce([fx / norm, fy / norm, 0.0], False)
            self.advance_ball(launch)
            if self.supervisor.step(self.basic_ms) == -1:
                self.closed = True
                raise RuntimeError("Webots simulation ended")
            self.world = self.read()
            self.last_substep_worlds.append(self.world)
        self.kick_held = {k for k, c in commands.items() if c.kick > 0}
        return self.world

    def close(self):
        # Runtime owner decides whether to quit Webots; Gym close doesn't destroy shared app state.
        pass
