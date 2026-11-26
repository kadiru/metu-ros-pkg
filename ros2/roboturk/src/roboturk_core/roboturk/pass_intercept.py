"""Raise a defensive response when an applied pass creates a reachable lane."""

from dataclasses import dataclass, replace
import math

from .carpet import SWITCH_RATIO, ground_step
from .model import Command, Referee, RobotKey, body_vector, possession, unit, wrap
from .timed_motion import TIMES, Leg, clear, inside, interpolation_margin


MIN_BALL_SPEED = 0.45
SETUP_MARGIN = 0.04
REASSIGN_MARGIN = 0.16
HEADING_TOLERANCE = 0.28


@dataclass(frozen=True)
class Interception:
    actor: RobotKey
    center: tuple[float, float]
    contact: tuple[float, float]
    heading: float
    seconds: float
    slack: float
    leg: Leg

    def command(self, world, robot, cfg, receive):
        gap = math.hypot(world.ball.x - robot.x, world.ball.y - robot.y)
        centered = math.dist((robot.x, robot.y), self.center) <= 0.10
        if possession(world, robot.key, cfg):
            return Command(dribbler=1)
        if receive and gap <= 0.38 and (centered or self.seconds <= 0.14):
            return replace(receive, opening=0).command(world, robot, self.contact)
        vx, vy = body_vector(*self.leg.velocity(cfg.dt), robot.yaw)
        error = wrap(self.heading - robot.yaw)
        yaw_rate = max(-cfg.max_omega, min(cfg.max_omega, 8 * error))
        return Command(vx, vy, yaw_rate, 1)

    def reason(self):
        return (
            f"Measured ball motion predicts a reachable meeting point in {self.seconds:.2f} s."
        )


def turn_time(error, limit):
    """Time to turn inside the roller's yaw tolerance under the approach command."""
    start = abs(error)
    if start <= HEADING_TOLERANCE:
        return 0.0
    knee = max(limit / 8, HEADING_TOLERANCE)
    return max(0.0, start - knee) / limit + math.log(min(start, knee) / HEADING_TOLERANCE) / 8


def intercept_pass(world, flight, cfg, clearance, observed, previous=None):
    """Find one field player who can arrive and turn before the observed ground pass."""
    ball = world.ball
    speed = math.hypot(ball.vx, ball.vy)
    if (
        world.referee != Referee.RUNNING
        or not ball.visible
        or speed < MIN_BALL_SPEED
        or ball.z > cfg.ball_radius + 0.015
        or abs(ball.vz) > 0.10
    ):
        return None
    ux, uy = unit(ball.vx, ball.vy)
    heading = math.atan2(-uy, -ux)
    switch = min(speed, flight.launch_speed * SWITCH_RATIO) if flight.launch_speed else speed
    margin = interpolation_margin(cfg)
    players = [
        r
        for r in world.robots
        if r.visible and r.key.id != 0 and r.key.team != flight.team and inside((r.x, r.y))
    ]
    turns = {r.key: turn_time(wrap(heading - r.yaw), cfg.max_omega) for r in players}
    tracks = {}
    for r in world.robots:
        if r.visible:
            vx, vy = observed.get(r.key, (r.vx, r.vy))
            tracks[r.key] = [(r.x + t * vx, r.y + t * vy) for t in TIMES]

    best = None
    for n, seconds in enumerate(TIMES[1:], 1):
        if seconds > flight.expires - world.time or (
            best and seconds > best.seconds + REASSIGN_MARGIN
        ):
            break
        distance, arrival_speed = ground_step(speed, switch, seconds)
        if arrival_speed < MIN_BALL_SPEED:
            break
        contact = (ball.x + distance * ux, ball.y + distance * uy)
        center = (contact[0] + cfg.possession_x * ux, contact[1] + cfg.possession_x * uy)
        if not inside(center):
            continue
        ready = []
        for r in players:
            incumbent = previous is not None and r.key == previous.actor
            deadline = seconds if incumbent else seconds - SETUP_MARGIN
            if turns[r.key] > deadline:
                continue
            leg = Leg.to((r.x, r.y), (r.vx, r.vy), center, cfg=cfg)
            slack = seconds - max(leg.duration, turns[r.key])
            if slack < (0 if incumbent else SETUP_MARGIN):
                continue
            path = [leg.position(t) for t in TIMES[: n + 1]]
            if not clear(
                path,
                (track for key, track in tracks.items() if key != r.key),
                clearance,
                margin=margin,
            ):
                continue
            candidate = Interception(r.key, center, contact, heading, seconds, slack, leg)
            if incumbent:
                return candidate
            ready.append(candidate)
        if ready and best is None:
            best = max(ready, key=lambda plan: (plan.slack, -plan.actor.id))
            if previous is None:
                break
    return best
