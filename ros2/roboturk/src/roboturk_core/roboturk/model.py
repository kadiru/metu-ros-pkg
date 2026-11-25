from dataclasses import asdict, dataclass, field, replace
from functools import lru_cache
from enum import StrEnum
import math

from . import field as pitch
from .carpet import BALL_RADIUS

KICKER_LIMIT = 10.0
SSL_KICK_LIMIT = 6.5
MAX_KICK_ELEVATION = math.pi / 4
GRAVITY = 9.81
ROBOT_HEIGHT = 0.15


class Skill(StrEnum):
    MOVE = "move"
    CHASE = "chase"
    DRIBBLE = "dribble"
    PASS = "pass"
    RECEIVE = "receive"
    SHOOT = "shoot"
    HOLD = "hold"


class Referee(StrEnum):
    HALT = "halt"
    STOP = "stop"
    RUNNING = "running"


@dataclass(frozen=True, order=True)
class RobotKey:
    team: str
    id: int

    def __post_init__(self):
        if self.team not in ("blue", "yellow") or not 0 <= self.id < 16:
            raise ValueError("Expected blue/yellow and robot ID 0..15")


@dataclass(frozen=True)
class Robot:
    key: RobotKey
    x: float
    y: float
    yaw: float = 0.0
    vx: float = 0.0  # world frame, m/s
    vy: float = 0.0
    omega: float = 0.0
    visible: bool = True


@dataclass(frozen=True)
class Ball:
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    visible: bool = True
    z: float = BALL_RADIUS  # height of the center above the carpet, m
    vz: float = 0.0


@dataclass(frozen=True)
class Config:
    length: float = pitch.LENGTH
    width: float = pitch.WIDTH
    goal_width: float = pitch.GOAL_WIDTH
    robot_radius: float = 0.09
    ball_radius: float = BALL_RADIUS
    ball_mass: float = 0.046
    max_speed: float = 3.0
    max_omega: float = 8.0
    max_kick: float = KICKER_LIMIT
    dt: float = 0.02
    max_steps: int = 500
    neighbors: int = 5
    possession_x: float = 0.115
    max_acceleration: float = 5.0

    def __post_init__(self):
        if not math.isfinite(self.max_acceleration) or self.max_acceleration <= 0:
            raise ValueError("max_acceleration must be finite and positive")

    @property
    def ball_exit_x(self):
        return (self.length + pitch.LINE_WIDTH) / 2 + self.ball_radius

    @property
    def ball_exit_y(self):
        return (self.width + pitch.LINE_WIDTH) / 2 + self.ball_radius


@lru_cache(maxsize=64)
def skill_limits(cfg):
    """The published specialist actors encode 2 m/s robot and 5 m/s kick scales."""
    return replace(cfg, max_speed=min(cfg.max_speed, 2.0), max_kick=min(cfg.max_kick, 5.0))


def compatible_policy_config(saved, cfg):
    if saved == asdict(cfg):
        return True
    # Older artifacts predate acceleration limiting. Preserve their input/output SI scales
    # and continue to reject changes to geometry, control period or observation layout.
    legacy = asdict(skill_limits(cfg))
    del legacy["max_acceleration"]
    return saved == legacy


@dataclass(frozen=True)
class World:
    time: float
    robots: tuple[Robot, ...]
    ball: Ball
    referee: Referee = Referee.HALT
    attack_sign: int = 1  # direction blue attacks; yellow attacks the opposite goal

    def __post_init__(self):
        if self.attack_sign not in (-1, 1):
            raise ValueError("attack_sign must be -1 or +1")
        if len({r.key for r in self.robots}) != len(self.robots):
            raise ValueError("Duplicate robot identity")

    def robot(self, key: RobotKey) -> Robot:
        return next(r for r in self.robots if r.key == key)

    def direction(self, team: str) -> int:
        return self.attack_sign if team == "blue" else -self.attack_sign


@dataclass(frozen=True)
class Command:
    vx: float = 0.0  # robot frame, m/s; x-forward, y-left
    vy: float = 0.0
    omega: float = 0.0
    dribbler: float = 0.0  # duty 0..1
    kick: float = 0.0  # total 3D launch speed in m/s, edge-triggered by backend
    kick_elevation: float = 0.0  # launch angle above the carpet, radians


@dataclass(frozen=True)
class Intent:
    skill: Skill
    target: tuple[float, float]
    receiver: RobotKey | None = None


@dataclass
class Play:
    name: str
    assignments: dict[RobotKey, Intent] = field(default_factory=dict)


def unit(dx, dy):
    length = max(1e-6, math.hypot(dx, dy))
    return dx / length, dy / length


def body_vector(x: float, y: float, yaw: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x + s * y, -s * x + c * y


def world_vector(x: float, y: float, yaw: float) -> tuple[float, float]:
    return body_vector(x, y, -yaw)


def drive_velocity(robot, command, dt, cfg):
    """Slew the bounded body request from measured world velocity, including braking."""
    tx, ty = world_vector(command.vx, command.vy, robot.yaw)
    dx, dy = tx - robot.vx, ty - robot.vy
    scale = max(1.0, math.hypot(dx, dy) / (cfg.max_acceleration * dt))
    return robot.vx + dx / scale, robot.vy + dy / scale


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def launch_velocity(speed, yaw, elevation):
    planar = speed * math.cos(elevation)
    return planar * math.cos(yaw), planar * math.sin(yaw), speed * math.sin(elevation)


def possession(world: World, key: RobotKey, cfg: Config) -> bool:
    robot = world.robot(key)
    bx, by = body_vector(world.ball.x - robot.x, world.ball.y - robot.y, robot.yaw)
    return (
        robot.visible
        and world.ball.visible
        and world.ball.z <= cfg.ball_radius + 0.012
        and abs(bx - cfg.possession_x) < 0.035
        and abs(by) < 0.045
    )


def bounded(command: Command, cfg: Config) -> Command:
    values = (
        command.vx,
        command.vy,
        command.omega,
        command.dribbler,
        command.kick,
        command.kick_elevation,
    )
    if not all(math.isfinite(v) for v in values):
        return Command()
    scale = max(1.0, math.hypot(command.vx, command.vy) / cfg.max_speed)
    return Command(
        command.vx / scale,
        command.vy / scale,
        max(-cfg.max_omega, min(cfg.max_omega, command.omega)),
        max(0.0, min(1.0, command.dribbler)),
        max(0.0, min(KICKER_LIMIT, cfg.max_kick, command.kick)),
        max(0.0, min(MAX_KICK_ELEVATION, command.kick_elevation)) if command.kick > 0 else 0.0,
    )


def constrain_skill(command: Command, skill: Skill) -> Command:
    """Shared actuator mask for training and deployment."""
    if skill == Skill.HOLD:
        return Command()
    kick = command.kick if skill in (Skill.PASS, Skill.SHOOT) else 0.0
    dribbler = command.dribbler if skill != Skill.MOVE else 0.0
    return replace(
        command,
        dribbler=dribbler,
        kick=kick,
        kick_elevation=command.kick_elevation if kick else 0.0,
    )


def safe_command(
    world: World,
    key: RobotKey,
    command: Command,
    cfg: Config,
    state_age: float = 0.0,
    referee_age: float = 0.0,
) -> Command:
    # STOP conservatively holds all robots. Set-piece movement comes later.
    if (
        world.referee != Referee.RUNNING
        or not 0 <= state_age <= 0.25
        or not 0 <= referee_age <= 0.5
        or not world.robot(key).visible
    ):
        return Command()
    return bounded(command, cfg)
