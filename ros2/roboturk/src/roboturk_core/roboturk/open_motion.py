"""Team-local steering from consecutive overhead observations in open play."""

from dataclasses import dataclass, replace
import math

from .touchline import END_LIMIT, SIDE_LIMIT, brake_axis
from .model import body_vector, world_vector
from .five_roles import CFG
from .goal_cover import GOAL_CENTER_LIMIT, GOAL_CHANNEL_HALF, SAVE_SPEED
from .placement_traffic import separation
from .tackle import ROBOT_GAP

FETCH_SPEED = 0.42
QUALIFIED_PACE = 1.2


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


def safe_fetch_velocity(vx, vy, nearby, dt, *, acceleration=CFG.max_acceleration):
    """Check the swept gap and room to brake before accepting a close approach."""
    for dx, dy, qx, qy, _, _ in nearby:
        distance = math.hypot(dx, dy)
        if distance < 1e-8:
            return False
        rx, ry = vx - qx, vy - qy
        closing = max(0, -(dx * rx + dy * ry) / distance)
        # Reserve half the available braking authority for the other chaser.
        if closing * dt + closing**2 / acceleration > max(0, distance - ROBOT_GAP):
            return False
        if separation(dx, dy, rx, ry, 0.18) < min(distance, ROBOT_GAP) - 1e-8:
            return False
    return True


def steer_team(
    robots,
    team,
    requests,
    observed,
    policy,
    dt,
    recovering=(),
    challenges=None,
    goal_cover=None,
    keeper_handoffs=None,
    fetching=(),
    cfg=CFG,
    expand_pace=False,
    turning=(),
    interceptors=(),
):
    """Translate this team's requests using only measured motion of surrounding players."""
    own = [r for r in robots if r.key.team == team]
    if set(requests) != {r.key for r in own}:
        raise ValueError("Expected exactly one team's velocity requests")
    speed = min(QUALIFIED_PACE, cfg.max_speed)
    group_scale = speed / max(speed, *(math.hypot(c.vx, c.vy) for c in requests.values()))
    reach = 3.1
    if expand_pace:
        reach = max(reach, cfg.max_speed + max(math.hypot(*v) for v in observed.values()))
    out = {}
    for r in own:
        cmd = requests[r.key]
        carrier = challenges.get(r.key) if challenges else None
        yielding = keeper_handoffs.get(r.key) if keeper_handoffs else None
        intercepting = yielding is not None and r.key.id == 0
        fetch = r.key in fetching
        scale = (
            cfg.max_speed / max(cfg.max_speed, math.hypot(cmd.vx, cmd.vy))
            if expand_pace
            else speed / max(speed, math.hypot(cmd.vx, cmd.vy))
            if r.key in turning or r.key in interceptors
            else group_scale
        )
        x, y = world_vector(cmd.vx * scale, cmd.vy * scale, r.yaw)
        if fetch:
            reduce = FETCH_SPEED / max(FETCH_SPEED, math.hypot(x, y))
            x, y = x * reduce, y * reduce
        nearby = [
            (
                r.x - q.x,
                r.y - q.y,
                *observed[q.key],
                ROBOT_GAP
                if (carrier and q.key == carrier.key)
                or (fetch and math.hypot(*observed[q.key]) <= 0.45)
                else policy.clearance,
                min(policy.horizon, 0.20)
                if r.key in turning
                or (
                    yielding is not None
                    and (q.key == yielding or (carrier is not None and q.key == carrier.key))
                )
                else policy.horizon,
            )
            for q in robots
            if q.key != r.key
            and math.hypot(r.x - q.x, r.y - q.y) < policy.clearance + reach * policy.horizon
        ]

        def fast_lane_clear(vx, vy):
            if not expand_pace or math.hypot(vx, vy) <= QUALIFIED_PACE:
                return True
            for dx, dy, qx, qy, clearance, horizon in nearby:
                distance = math.hypot(dx, dy)
                relative = vx - qx, vy - qy
                if separation(dx, dy, *relative, horizon) < clearance:
                    return False
                # One reaction tick plus braking room at the proposed closing speed.
                closing = max(0, -(dx * relative[0] + dy * relative[1]) / max(distance, 1e-8))
                if closing * dt + closing**2 / cfg.max_acceleration > max(0, distance - clearance):
                    return False
            return True

        def loss(vx, vy):
            cost = (vx - x) ** 2 + (vy - y) ** 2
            for dx, dy, qx, qy, clearance, horizon in nearby:
                # Both partners update their crossing plans every tick. Keep the
                # same clearances while limiting extrapolation of the handoff.
                gap = separation(dx, dy, vx - qx, vy - qy, horizon)
                cost += 420 * max(0, clearance - gap) ** 2
                # When already inside the margin prefer to open a gap.
                end = math.hypot(dx + (vx - qx) * 0.16, dy + (vy - qy) * 0.16)
                cost += 120 * max(0, clearance - end) ** 2
            return cost

        if nearby and (fetch or loss(x, y) > 1e-8 or not fast_lane_clear(x, y)):
            options = [(x, y)]
            for speed in (1.0, 0.65, 0.30):
                for turn in (policy.turn, -policy.turn, 2 * policy.turn, -2 * policy.turn, 0):
                    c, s = math.cos(turn) * speed, math.sin(turn) * speed
                    options.append((c * x - s * y, s * x + c * y))
            options.append((0, 0))
            if expand_pace:
                options = [v for v in options if fast_lane_clear(*v)]
            if fetch:
                options = [
                    v
                    for v in options
                    if safe_fetch_velocity(*v, nearby, dt, acceleration=cfg.max_acceleration)
                ]
                # A converging robot can consume the available room on its own.
                # Brake rather than committing to a narrower lane in that tick.
                if not options:
                    options = [(0, 0)]
            x, y = min(options, key=lambda v: loss(*v))
        if carrier:
            dx, dy = r.x - carrier.x, r.y - carrier.y
            distance = max(1e-9, math.hypot(dx, dy))
            nx, ny = dx / distance, dy / distance
            cx, cy = observed[carrier.key]
            # Bound closing speed with braking distance plus one control tick.
            # The handed-off keeper may travel at its save speed while there
            # is stopping room. The 0.32 m/s tackle cap is for close draws.
            room = max(0, distance - ROBOT_GAP)
            ceiling = SAVE_SPEED if intercepting else 0.32
            authority = 4 * min(1, cfg.max_acceleration / CFG.max_acceleration)
            closing = min(ceiling, math.sqrt(authority * room) - 0.08)
            radial = (x - cx) * nx + (y - cy) * ny
            correction = max(0, -closing - radial)
            x, y = x + correction * nx, y + correction * ny
        if goal_cover and r.key in goal_cover:
            sign = goal_cover[r.key]
            # Leave enough lateral braking room before passing between the posts.
            brake = cfg.max_acceleration / math.sqrt(2)
            safe_y = abs(r.y) + abs(r.vy) * dt + r.vy**2 / (2 * brake)
            end = GOAL_CENTER_LIMIT if safe_y <= GOAL_CHANNEL_HALF else END_LIMIT
            own_x = (
                brake_axis(sign * r.x, sign * x, end, dt, acceleration=cfg.max_acceleration)
                if sign * x > 0
                else sign * x
            )
            x = sign * max(own_x, (-END_LIMIT - sign * r.x) / dt)
            y = brake_axis(
                r.y,
                y,
                GOAL_CHANNEL_HALF if sign * r.x > 4.28 or own_x > 0 else 2.85,
                dt,
                acceleration=cfg.max_acceleration,
            )
        elif r.key in recovering:
            x = brake_axis(r.x, x, END_LIMIT, dt, acceleration=cfg.max_acceleration)
            y = brake_axis(r.y, y, SIDE_LIMIT, dt, acceleration=cfg.max_acceleration)
        elif expand_pace:
            x = brake_axis(r.x, x, END_LIMIT, dt, acceleration=cfg.max_acceleration)
            y = brake_axis(r.y, y, 2.85, dt, acceleration=cfg.max_acceleration)
        else:
            x = max(min(0, (-4.35 - r.x) / dt), min(max(0, (4.35 - r.x) / dt), x))
            y = max(min(0, (-2.85 - r.y) / dt), min(max(0, (2.85 - r.y) / dt), y))
            if r.key.id == 0 and abs(r.x) > END_LIMIT:
                y = brake_axis(r.y, y, GOAL_CHANNEL_HALF, dt, acceleration=cfg.max_acceleration)
        vx, vy = body_vector(x, y, r.yaw)
        out[r.key] = replace(cmd, vx=vx, vy=vy)
    return out
