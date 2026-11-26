"""Explicit play/role/skill orchestration around learned option implementations.

This coordinator is scripted. It does not silently extend the already-frozen
continuous self-play manager. Commitment prevents frame-to-frame role thrashing;
referee, visibility and deadline conditions still cancel immediately.
"""

from dataclasses import dataclass
import math
from .model import Referee, RobotKey, wrap
from .tdp_options import BallIntent, SPEED, OMEGA


@dataclass(frozen=True)
class ReceptionContract:
    identifier: str
    source: tuple[float, float]
    point: tuple[float, float]
    arrival_at: float
    expires_at: float


@dataclass(frozen=True)
class Assignment:
    robot: RobotKey
    role: str
    intent: BallIntent
    contract_id: str
    started_at: float
    expires_at: float


def segment_clearance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    fraction = max(
        0.0,
        min(
            1.0,
            ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
            / max(1e-12, dx * dx + dy * dy),
        ),
    )
    return math.hypot(point[0] - start[0] - fraction * dx, point[1] - start[1] - fraction * dy)


class TdpPlaybook:
    def __init__(self):
        self.current = None
        self.events = []

    def cancel(self, world, reason):
        if self.current:
            self.events.append(
                {
                    "time": world.time,
                    "event": "cancel",
                    "contract": self.current.contract_id,
                    "reason": reason,
                }
            )
        self.current = None

    def finish(self, world, outcome):
        if outcome not in ("success", "failure", "timeout"):
            raise ValueError(outcome)
        if self.current:
            self.events.append(
                {
                    "time": world.time,
                    "event": outcome,
                    "contract": self.current.contract_id,
                    "robot": {"team": self.current.robot.team, "id": self.current.robot.id},
                }
            )
        self.current = None

    def choose(self, world, team, candidates, plan=None):
        if world.referee != Referee.RUNNING or not world.ball.visible:
            self.cancel(world, "referee_or_ball_visibility")
            return None
        visible = {r.key: r for r in world.robots if r.key.team == team and r.visible}
        if self.current:
            if self.current.robot not in visible:
                self.cancel(world, "robot_visibility")
            elif world.time > self.current.expires_at:
                self.cancel(world, "deadline")
                return None
            else:
                return self.current
        robots = [visible[k] for k in sorted(set(candidates)) if k in visible]
        if not robots:
            return None
        sign = world.direction(team)
        if plan is None:
            if sign * world.ball.x >= -2.2:
                return None
            opponents = [r for r in world.robots if r.key.team != team and r.visible]
            outlets = [(sign * -0.6, sign * y) for y in (-1.7, 1.7)]
            origin = (world.ball.x, world.ball.y)
            outlet = max(
                outlets,
                key=lambda target: min(
                    (segment_clearance((r.x, r.y), origin, target) for r in opponents), default=10.0
                ),
            )
            heading = math.atan2(outlet[1] - origin[1], outlet[0] - origin[0])
            intent = BallIntent("clear", origin, heading, outlet)
            identifier = f"clear:{world.time:.3f}"
            deadline = world.time + 8
            role = "defensive_clearer"
        else:
            if plan.expires_at < world.time or abs(plan.point[0]) > 4.1 or abs(plan.point[1]) > 2.6:
                return None
            incoming = math.atan2(plan.source[1] - plan.point[1], plan.source[0] - plan.point[0])
            goal = (sign * 4.5, 0.0)
            shot = math.atan2(goal[1] - plan.point[1], goal[0] - plan.point[0])
            lane_open = all(
                segment_clearance((r.x, r.y), plan.point, goal) > 0.22
                for r in world.robots
                if r.key not in candidates
            )
            redirect = sign * plan.point[0] > 0.8 and abs(wrap(incoming - shot)) < 1.1 and lane_open
            intent = BallIntent(
                "redirect" if redirect else "receive",
                plan.point,
                shot if redirect else incoming,
                goal,
            )
            identifier, deadline = plan.identifier, plan.expires_at
            role = "redirect_receiver" if redirect else "settling_receiver"

        def cost(r):
            target = (
                intent.point[0] - 0.115 * math.cos(intent.heading),
                intent.point[1] - 0.115 * math.sin(intent.heading),
            )
            return (
                math.hypot(target[0] - r.x, target[1] - r.y) / SPEED
                + abs(wrap(intent.heading - r.yaw)) / OMEGA
            )

        robot = min(robots, key=lambda r: (cost(r), r.key))
        if plan is not None and world.time + cost(robot) > 0.05 + plan.arrival_at:
            return None
        self.current = Assignment(robot.key, role, intent, identifier, world.time, deadline)
        self.events.append(
            {
                "time": world.time,
                "event": "commit",
                "contract": identifier,
                "role": role,
                "kind": intent.kind,
                "robot": {"team": robot.key.team, "id": robot.key.id},
                "point": intent.point,
                "heading": intent.heading,
                "outlet": intent.outlet,
                "expires_at": deadline,
            }
        )
        return self.current
