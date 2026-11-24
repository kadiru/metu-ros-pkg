"""Team-local steering from consecutive overhead observations in open play."""

from dataclasses import dataclass, replace
import math

from .touchline import END_LIMIT, SIDE_LIMIT, brake_axis
from .model import body_vector, world_vector
from .placement_traffic import separation
from .tackle import ROBOT_GAP


@dataclass(frozen=True)
class Steering:
    horizon: float
    clearance: float
    turn: float

    def __post_init__(self):
        if not (
            0.08 <= self.horizon <= 1 and 0.18 <= self.clearance <= 0.5 and 0.15 <= self.turn <= 1.3
        ):
            raise ValueError("Open-play steering weights outside supported range")


def observed_velocity(robots, previous, dt):
    """Causal velocities from two position samples. A new kickoff starts at rest."""
    return {
        r.key: ((r.x - previous[r.key][0]) / dt, (r.y - previous[r.key][1]) / dt)
        if r.key in previous
        else (0.0, 0.0)
        for r in robots
    }


def steer_team(robots, team, requests, observed, policy, dt, recovering=(), challenges=None):
    """Translate this team's requests using only measured motion of surrounding players."""
    own = [r for r in robots if r.key.team == team]
    if set(requests) != {r.key for r in own}:
        raise ValueError("Expected exactly one team's velocity requests")
    scale = 1.2 / max(1.2, *(math.hypot(c.vx, c.vy) for c in requests.values()))
    out = {}
    for r in own:
        cmd = requests[r.key]
        carrier = challenges.get(r.key) if challenges else None
        x, y = world_vector(cmd.vx * scale, cmd.vy * scale, r.yaw)
        nearby = [
            (
                r.x - q.x,
                r.y - q.y,
                *observed[q.key],
                ROBOT_GAP if carrier and q.key == carrier.key else policy.clearance,
            )
            for q in robots
            if q.key != r.key
            and math.hypot(r.x - q.x, r.y - q.y) < policy.clearance + 3.1 * policy.horizon
        ]

        def loss(vx, vy):
            cost = (vx - x) ** 2 + (vy - y) ** 2
            for dx, dy, qx, qy, clearance in nearby:
                gap = separation(dx, dy, vx - qx, vy - qy, policy.horizon)
                cost += 420 * max(0, clearance - gap) ** 2
                # When already inside the margin prefer to open a gap.
                end = math.hypot(dx + (vx - qx) * 0.16, dy + (vy - qy) * 0.16)
                cost += 120 * max(0, clearance - end) ** 2
            return cost

        if nearby and loss(x, y) > 1e-8:
            options = [(x, y)]
            for speed in (1.0, 0.65, 0.30):
                for turn in (policy.turn, -policy.turn, 2 * policy.turn, -2 * policy.turn, 0):
                    c, s = math.cos(turn) * speed, math.sin(turn) * speed
                    options.append((c * x - s * y, s * x + c * y))
            options.append((0, 0))
            x, y = min(options, key=lambda v: loss(*v))
        if carrier:
            dx, dy = r.x - carrier.x, r.y - carrier.y
            distance = max(1e-9, math.hypot(dx, dy))
            nx, ny = dx / distance, dy / distance
            cx, cy = observed[carrier.key]
            # Bound closing speed with braking distance plus one control tick.
            room = max(0, distance - ROBOT_GAP)
            closing = min(0.32, math.sqrt(4 * room) - 0.08)
            radial = (x - cx) * nx + (y - cy) * ny
            correction = max(0, -closing - radial)
            x, y = x + correction * nx, y + correction * ny
        if r.key in recovering:
            x = brake_axis(r.x, x, END_LIMIT, dt)
            y = brake_axis(r.y, y, SIDE_LIMIT, dt)
        else:
            x = max(min(0, (-4.35 - r.x) / dt), min(max(0, (4.35 - r.x) / dt), x))
            y = max(min(0, (-2.85 - r.y) / dt), min(max(0, (2.85 - r.y) / dt), y))
        vx, vy = body_vector(x, y, r.yaw)
        out[r.key] = replace(cmd, vx=vx, vy=vy)
    return out
