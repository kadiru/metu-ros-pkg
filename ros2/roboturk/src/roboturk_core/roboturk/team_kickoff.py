"""A private formation and causal opponent observations for each kickoff team."""

import math
from dataclasses import replace
from itertools import combinations

from .kickoff_drive import SPEED, TOLERANCE, path_to
from .model import Command, Robot, RobotKey, body_vector, world_vector, wrap
from .open_motion import Steering, observed_velocity, steer_team

# Measured robot centers are extrapolated only over this short steering horizon.
STEERING = Steering(horizon=0.62, clearance=0.34, turn=0.57)


def limit_approach(robot, command, seen):
    """Keep each robot's share of closing speed within the observed free distance."""
    preferred = world_vector(command.vx, command.vy, robot.yaw)
    constraints = []
    for q in seen:
        if q.key == robot.key:
            continue
        dx, dy = q.x - robot.x, q.y - robot.y
        distance = math.hypot(dx, dy)
        # Each robot can consume half the gap to 29 cm over the next 0.24 s.
        limit = max(0.0, (distance - 0.29) / 0.48)
        if limit < SPEED and distance > 1e-9:
            constraints.append((dx / distance, dy / distance, limit))

    def feasible(v):
        return all(x * v[0] + y * v[1] <= cap + 1e-10 for x, y, cap in constraints)

    if feasible(preferred):
        return command
    # The Euclidean projection lies on an edge or at the intersection of two edges.
    choices = [(0.0, 0.0)]
    for x, y, cap in constraints:
        excess = x * preferred[0] + y * preferred[1] - cap
        projected = preferred[0] - x * excess, preferred[1] - y * excess
        if feasible(projected):
            choices.append(projected)
    for (x, y, cap), (u, v, other) in combinations(constraints, 2):
        det = x * v - y * u
        if abs(det) > 1e-9:
            crossing = (cap * v - y * other) / det, (x * other - cap * u) / det
            if feasible(crossing):
                choices.append(crossing)
    vx, vy = min(choices, key=lambda p: math.dist(p, preferred))
    bx, by = body_vector(vx, vy, robot.yaw)
    return replace(command, vx=bx, vy=by)


class TeamKickoffPlanner:
    def __init__(self, targets):
        teams = {r.key.team for r in targets}
        if len(teams) != 1:
            raise ValueError("Kickoff targets must belong to a single team")
        self.team = teams.pop()
        self.targets = {r.key: r for r in targets}
        self.previous = {}
        self.previous_time = None
        self.paths = {}
        self.next_plan = -math.inf

    def commands(self, own, opponent_xy, stamp, dt):
        if {r.key for r in own} != self.targets.keys() or any(
            k.team == self.team for k in opponent_xy
        ):
            raise ValueError("Expected own roster and opponent camera positions")
        others = tuple(Robot(k, x, y) for k, (x, y) in opponent_xy.items())
        seen = (*own, *others)
        sample_dt = stamp - self.previous_time if self.previous_time is not None else dt
        velocities = observed_velocity(
            seen, self.previous if sample_dt > 0 else {}, max(sample_dt, dt / 100)
        )
        self.previous = {r.key: (r.x, r.y) for r in seen}
        self.previous_time = stamp
        if stamp >= self.next_plan:
            # Route around slow players. Moving traffic is handled again on every control tick.
            parked = {r.key: (r.x, r.y) for r in seen if math.hypot(*velocities[r.key]) < 0.18}
            for r in own:
                goal = self.targets[r.key]
                dest = goal.x, goal.y
                obstacles = [p for k, p in parked.items() if k != r.key]
                self.paths[r.key] = path_to((r.x, r.y), dest, obstacles) or [dest]
            self.next_plan = stamp + 0.40

        requests = {}
        ready = True
        for r in own:
            goal = self.targets[r.key]
            path = self.paths[r.key]
            while len(path) > 1 and math.dist((r.x, r.y), path[0]) < 0.05:
                path.pop(0)
            dx, dy = path[0][0] - r.x, path[0][1] - r.y
            distance = math.hypot(goal.x - r.x, goal.y - r.y)
            if distance <= TOLERANCE:
                dx = dy = 0.0
            scale = max(dt, math.hypot(dx, dy) / SPEED)
            vx, vy = body_vector(dx / scale, dy / scale, r.yaw)
            angle = wrap(goal.yaw - r.yaw)
            requests[r.key] = Command(vx, vy, max(-2.0, min(2.0, angle / dt)))
            ready &= (
                distance <= TOLERANCE
                and abs(angle) < 0.015
                and math.hypot(r.vx, r.vy) < 0.035
                and abs(r.omega) < 0.08
            )
        steered = steer_team(seen, self.team, requests, velocities, STEERING, dt)
        return {r.key: limit_approach(r, steered[r.key], seen) for r in own}, ready


class IndependentKickoffs:
    """Dispatch one observation per side, then combine commands and readiness for Webots."""

    def __init__(self, formation):
        self.targets = {r.key: r for r in formation.robots}
        self.planners = {
            team: TeamKickoffPlanner(tuple(r for r in formation.robots if r.key.team == team))
            for team in ("blue", "yellow")
        }

    def commands(self, world, dt, frame):
        output = {}
        ready = True
        for team, planner in self.planners.items():
            own = tuple(r for r in world.robots if r.key.team == team)
            other = "yellow" if team == "blue" else "blue"
            bots = frame.robots_yellow if team == "blue" else frame.robots_blue
            positions = {RobotKey(other, b.robot_id): (b.x / 1000, b.y / 1000) for b in bots}
            commands, settled = planner.commands(own, positions, frame.t_capture, dt)
            output.update(commands)
            ready &= settled
        return output, ready
