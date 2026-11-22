from dataclasses import dataclass, field
from enum import StrEnum
import math


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


@dataclass(frozen=True)
class Config:
    # Training field defaults; not a complete implementation of SSL competition rules.
    length: float = 9.0
    width: float = 6.0
    goal_width: float = 1.0
    robot_radius: float = 0.09
    ball_radius: float = 0.0215
    ball_mass: float = 0.046
    max_speed: float = 2.0
    max_omega: float = 8.0
    max_kick: float = 5.0
    dt: float = 0.02
    max_steps: int = 500
    neighbors: int = 5
    possession_x: float = 0.115


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
    kick: float = 0.0  # target ball speed m/s, edge-triggered by backend


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


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def possession(world: World, key: RobotKey, cfg: Config) -> bool:
    robot = world.robot(key)
    bx, by = body_vector(world.ball.x - robot.x, world.ball.y - robot.y, robot.yaw)
    return (
        robot.visible
        and world.ball.visible
        and abs(bx - cfg.possession_x) < 0.035
        and abs(by) < 0.045
    )


def bounded(command: Command, cfg: Config) -> Command:
    values = (command.vx, command.vy, command.omega, command.dribbler, command.kick)
    if not all(math.isfinite(v) for v in values):
        return Command()
    scale = max(1.0, math.hypot(command.vx, command.vy) / cfg.max_speed)
    return Command(
        command.vx / scale,
        command.vy / scale,
        max(-cfg.max_omega, min(cfg.max_omega, command.omega)),
        max(0.0, min(1.0, command.dribbler)),
        max(0.0, min(cfg.max_kick, command.kick)),
    )


def constrain_skill(command: Command, skill: Skill) -> Command:
    """Shared actuator mask for training and deployment."""
    if skill == Skill.HOLD:
        return Command()
    kick = command.kick if skill in (Skill.PASS, Skill.SHOOT) else 0.0
    dribbler = command.dribbler if skill != Skill.MOVE else 0.0
    return Command(command.vx, command.vy, command.omega, dribbler, kick)


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
