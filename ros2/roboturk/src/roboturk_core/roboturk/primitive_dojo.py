"""Goal-conditioned arrival, interception, and transfer policies for isolated drills."""

from dataclasses import dataclass, replace
import math
import random

from .carpet import ROLL_DECEL
from .carrier_motor import MOTOR, track_roller
from .five_receive import FiveReceive
from .model import Ball, Command, Config, Referee, Robot, RobotKey, World, body_vector, bounded
from .model import possession, unit, wrap


CFG = Config(dt=0.04, max_kick=6.5)
TASKS = ("arrival", "intercept", "pass", "receive_race")
ACTOR = RobotKey("blue", 1)
RECEIVER = RobotKey("blue", 2)
DEFENDER = RobotKey("yellow", 1)
SERVERS = (RobotKey("yellow", 2), RobotKey("yellow", 3))
RECEIVE = FiveReceive(0.65, 6.0, 0)


def xy(entity):
    return entity.x, entity.y


@dataclass(frozen=True)
class Arrival:
    """Field position and yaw with a terminal field-frame velocity, in SI units."""

    point: tuple[float, float]
    yaw: float
    velocity: tuple[float, float] = (0, 0)

    @classmethod
    def at_speed(cls, point, yaw, speed=0.0, travel_heading=None):
        if not math.isfinite(speed) or speed < 0:
            raise ValueError("Arrival speed must be finite and nonnegative")
        heading = yaw if travel_heading is None else travel_heading
        return cls(point, yaw, (speed * math.cos(heading), speed * math.sin(heading)))


@dataclass
class ArrivalRoute:
    """Approach from behind the requested velocity when the start is off-axis."""

    target: Arrival
    entry: Arrival | None

    @classmethod
    def from_pose(cls, robot, target, cfg=CFG):
        magnitude = math.hypot(*target.velocity)
        if magnitude == 0:
            return cls(target, None)
        ux, uy = unit(*target.velocity)
        dx, dy = target.point[0] - robot.x, target.point[1] - robot.y
        along, cross = dx * ux + dy * uy, dx * uy - dy * ux
        runup = max(0.55, 0.32 + magnitude**2 / (2 * cfg.max_acceleration))
        if along >= runup and abs(cross) < 0.8 * along:
            return cls(target, None)
        point = target.point[0] - runup * ux, target.point[1] - runup * uy
        return cls(target, Arrival(point, target.yaw))

    def command(self, robot, cfg=CFG, **profile):
        if self.entry and math.dist(xy(robot), self.entry.point) <= 0.06:
            self.entry = None
        return go_to(robot, self.entry or self.target, cfg, **profile)


def go_to(robot, target, cfg=CFG, *, gain=3.2, speed=1.8, terminal=True):
    """Track a target pose, adding the requested terminal velocity as feed-forward."""
    dx, dy = target.point[0] - robot.x, target.point[1] - robot.y
    vx, vy = target.velocity if terminal else (0, 0)
    vx, vy = body_vector(vx + gain * dx, vy + gain * dy, robot.yaw)
    scale = min(1.0, speed / max(0.001, math.hypot(vx, vy)))
    return bounded(Command(vx * scale, vy * scale, 5 * wrap(target.yaw - robot.yaw)), cfg)


def ground_prediction(ball, seconds):
    """Rolling extrapolation for an observed ground ball after the initial slip."""
    speed = math.hypot(ball.vx, ball.vy)
    ux, uy = unit(ball.vx, ball.vy)
    t = min(seconds, speed / ROLL_DECEL)
    travel = t * (speed - 0.5 * t * ROLL_DECEL)
    remain = speed - t * ROLL_DECEL
    return (ball.x + travel * ux, ball.y + travel * uy), (remain * ux, remain * uy)


def deflection_station(world, robot, outlet, *, speed, settle, restitution, normal_bias=0):
    """Find a reachable stationary contact pose whose predicted rebound faces the outlet."""
    ball = world.ball
    incoming = math.atan2(ball.vy, ball.vx)
    radius = CFG.robot_radius + CFG.ball_radius
    best = None
    for step in range(3, 23):
        t = 0.08 * step
        impact, velocity = ground_prediction(ball, t)
        if math.hypot(*velocity) < 0.25:
            break
        aim = math.atan2(outlet[1] - impact[1], outlet[0] - impact[0])
        turn = wrap(aim - incoming)
        side = -1 if turn > 0 else 1
        choices = []
        for degrees in range(4, 89, 2):
            normal_angle = incoming + side * math.radians(degrees)
            nx, ny = math.cos(normal_angle), math.sin(normal_angle)
            impulse = (1 + restitution) * (velocity[0] * nx + velocity[1] * ny)
            vx, vy = velocity[0] - impulse * nx, velocity[1] - impulse * ny
            cost = abs(wrap(math.atan2(vy, vx) - aim)) + 0.3 * max(0, 0.4 - math.hypot(vx, vy))
            choices.append((cost, nx, ny))
        _, nx, ny = min(choices)
        normal_angle = math.atan2(ny, nx) + side * math.radians(normal_bias)
        nx, ny = math.cos(normal_angle), math.sin(normal_angle)
        point = impact[0] + radius * nx, impact[1] + radius * ny
        if abs(point[0]) >= 4.25 or abs(point[1]) >= 2.75:
            break
        eta = math.dist(xy(robot), point) / speed + speed / CFG.max_acceleration
        value = t + 5 * max(0, eta + settle - t)
        candidate = Arrival(point, math.atan2(-ny, -nx))
        if best is None or value < best[0]:
            best = value, candidate
        if eta + settle <= t:
            break
    return best[1] if best else Arrival(xy(robot), robot.yaw)


def intercept_target(world, robot, *, speed=1.8, margin=0.05, outgoing=None, cushion=0.45):
    """Choose a reachable point on the rolling path and specify contact pose/speed."""
    ball = world.ball
    ux, uy = unit(ball.vx, ball.vy)
    if math.hypot(ball.vx, ball.vy) < 0.08:
        ux, uy = unit(robot.x - ball.x, robot.y - ball.y)
    if outgoing is None:
        nx, ny = ux, uy
        contact = CFG.possession_x
        yaw = math.atan2(-ny, -nx)
    else:
        ox, oy = unit(outgoing[0] - ball.x, outgoing[1] - ball.y)
        nx, ny = unit(ux - ox, uy - oy)
        contact = CFG.robot_radius + CFG.ball_radius - 0.006
        yaw = math.atan2(-ny, -nx)
    best = None
    for i in range(1, 23):
        t = i * 0.08
        point, velocity = ground_prediction(ball, t)
        center = point[0] + contact * nx, point[1] + contact * ny
        if abs(center[0]) > 4.25 or abs(center[1]) > 2.75:
            break
        dx, dy = unit(center[0] - robot.x, center[1] - robot.y)
        closing = robot.vx * dx + robot.vy * dy
        eta = math.dist(xy(robot), center) / speed + max(0, speed - closing) / (
            2 * CFG.max_acceleration
        )
        if outgoing is None:
            terminal = cushion * velocity[0], cushion * velocity[1]
        else:
            terminal = -0.24 * nx, -0.24 * ny
        candidate = Arrival(center, yaw, terminal)
        value = t + 4 * max(0, eta + margin - t)
        if best is None or value < best[0]:
            best = value, candidate
        if eta + margin <= t:
            break
    return best[1] if best else Arrival(xy(robot), yaw)


def intercept(world, robot, profile, outgoing=None):
    ball = world.ball
    gap = math.dist(xy(robot), xy(ball))
    if outgoing is None and gap < profile.get("cushion_range", 0.44):
        receiver = FiveReceive(profile.get("receive_fraction", RECEIVE.cushion), RECEIVE.grip, 0)
        return receiver.command(world, robot, xy(ball))
    speed = profile.get("speed", 1.6)
    if profile.get("predict", False):
        target = intercept_target(
            world,
            robot,
            speed=speed,
            margin=profile.get("margin", 0.05),
            outgoing=outgoing,
            cushion=profile.get("arrival_fraction", 0.45),
        )
    else:
        yaw = math.atan2(ball.y - robot.y, ball.x - robot.x)
        radius = CFG.possession_x if outgoing is None else CFG.robot_radius * 0.7
        target = Arrival((ball.x - radius * math.cos(yaw), ball.y - radius * math.sin(yaw)), yaw)
    cmd = go_to(robot, target, speed=speed, gain=profile.get("gain", 4.6))
    return replace(cmd, dribbler=float(outgoing is None))


def pass_plan(ball, mate, profile):
    """Lead an observed moving receiver. Return a receiving point and kick speed."""
    distance = math.dist(xy(ball), xy(mate))
    guess = min(1.5, distance / 1.6)
    lead = guess * profile.get("lead", 0)
    target = (
        max(-4, min(4, mate.x + lead * mate.vx)),
        max(-2.55, min(2.55, mate.y + lead * mate.vy)),
    )
    distance = math.dist(xy(ball), target)
    return target, min(4.0, profile.get("base", 1.7) + profile.get("power", 0.23) * distance)


def pass_command(world, robot, target, speed):
    ball = world.ball
    aim = math.atan2(target[1] - ball.y, target[0] - ball.x)
    error = wrap(aim - robot.yaw)
    cmd = track_roller(world, robot, (0, 0), 5 * error, MOTOR)
    if possession(world, robot.key, CFG) and abs(error) < 0.045:
        return replace(cmd, kick=speed)
    return cmd


def serve_relay(world, next_server, speed):
    """Pulse the next kicker once its mouth can apply a kick."""
    return {
        key: Command(
            dribbler=1.0, kick=speed if key == next_server and possession(world, key, CFG) else 0
        )
        for key in SERVERS
    }


@dataclass(frozen=True)
class Scenario:
    task: str
    context: str
    world: World
    arrival: Arrival | None = None
    outlet: tuple[float, float] | None = None
    run_speed: float = 0.0
    serve_speed: float = 0.0
    rival_speed: float = 0.0


def scenario(task, seed):
    rng = random.Random(seed)
    side = rng.choice((-1, 1))
    if task == "arrival":
        moving = seed % 2 == 1
        axis = rng.uniform(-math.pi, math.pi)
        u = math.cos(axis), math.sin(axis)
        point = rng.uniform(-0.6, 0.6), rng.uniform(-0.5, 0.5)
        distance, offset = rng.uniform(0.9, 2.3), rng.uniform(-0.45, 0.45)
        x = point[0] - distance * u[0] - offset * u[1]
        y = point[1] - distance * u[1] + offset * u[0]
        robot = Robot(ACTOR, x, y, rng.uniform(-math.pi, math.pi))
        goal = Arrival.at_speed(
            point, rng.uniform(-math.pi, math.pi), rng.uniform(0.65, 1.45) if moving else 0, axis
        )
        return Scenario(
            task,
            "moving" if moving else "stop",
            World(0, (robot,), Ball(3.5, 2.5), Referee.RUNNING),
            arrival=goal,
        )
    if task == "intercept":
        redirect = seed % 2 == 1
        x, y = side * rng.uniform(-0.34, 0.26), side * rng.uniform(0.4, 0.82)
        learner = Robot(ACTOR, x, y, rng.uniform(-math.pi, math.pi))
        a = Robot(SERVERS[0], -side * 1.9, 0, 0 if side > 0 else math.pi)
        b = Robot(SERVERS[1], side * 1.9, 0, math.pi if side > 0 else 0)
        ball = Ball(a.x + side * CFG.possession_x, 0)
        goal = (side * 0.8, -side * 1.65) if redirect else None
        return Scenario(
            task,
            "deflect" if redirect else "trap",
            World(0, (learner, a, b), ball, Referee.RUNNING),
            outlet=goal,
            serve_speed=rng.uniform(1.95, 2.55),
        )
    if task not in ("pass", "receive_race"):
        raise ValueError(f"Unknown dojo task {task}")
    moving = task == "receive_race" or seed % 2 == 1
    x, y = rng.uniform(0.52, 1.12), side * rng.uniform(-0.3, 0.4)
    yaw = math.atan2(y, x + 1.3) + rng.uniform(-0.22, 0.22)
    passer = Robot(ACTOR, -1.3, 0, yaw)
    run = side * rng.uniform(0.48, 0.86) if moving else 0
    receiver = Robot(RECEIVER, x, y, math.pi, vy=0.6 * run)
    ball = Ball(passer.x + CFG.possession_x * math.cos(yaw), CFG.possession_x * math.sin(yaw))
    squad = [passer, receiver]
    opponent_speed = 0.0
    context = "on the run" if moving else "to feet"
    if task == "receive_race":
        fast = seed % 2 == 1
        context = "fast rival" if fast else "slow rival"
        squad.append(
            Robot(DEFENDER, rng.uniform(-0.36, 0.42), y + side * rng.uniform(0.73, 1.12), math.pi)
        )
        opponent_speed = rng.uniform(1.20, 1.55) if fast else rng.uniform(0.64, 0.92)
    return Scenario(
        task,
        context,
        World(0, tuple(squad), ball, Referee.RUNNING),
        run_speed=run,
        rival_speed=opponent_speed,
    )
