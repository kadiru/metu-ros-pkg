"""Bounded, receding-horizon trajectory search for 5v5 off-ball navigation."""

from dataclasses import dataclass, replace
import math

from . import field
from .five_roles import (
    CFG,
    INTERPOLATION_MARGIN as INTERPOLATION_MARGIN,
    PENALTY_FRONT,
    PENALTY_HALF_WIDTH,
    TIMED_ROLES,
    TRAJECTORY_SAMPLE_DT,
)
from .model import body_vector, world_vector
from .placement_traffic import separation
from .touchline import END_LIMIT

# Splitting the budgets between axes bounds the planar norms even on diagonal moves.
ACCEL = CFG.max_acceleration / math.sqrt(2)
QUALIFIED_PACE = 1.2
TRAVEL_CFG = replace(CFG, max_speed=QUALIFIED_PACE)
SPEED = QUALIFIED_PACE / math.sqrt(2)
INTERVAL = TRAJECTORY_SAMPLE_DT
TIMES = tuple(i * INTERVAL for i in range(16))
SIDE = field.WIDTH / 2 - 0.15


@dataclass(frozen=True)
class Axis:
    position: float
    velocity: float
    phases: tuple  # (duration, acceleration) from the measured state

    def at(self, t):
        p, v = self.position, self.velocity
        for duration, a in self.phases:
            step = min(t, duration)
            p += step * (v + a * step / 2)
            v += a * step
            t -= step
            if t <= 0:
                break
        return p, v

    @property
    def duration(self):
        return sum(duration for duration, _ in self.phases)


def axis_to(position, velocity, target, *, speed=SPEED, accel=ACCEL):
    """Time-optimal 1D acceleration, optional cruise, and braking to rest."""
    p, v = position, velocity
    phases = []
    if abs(v) > speed:
        capped = math.copysign(speed, v)
        t = abs(v - capped) / accel
        phases.append((t, -math.copysign(accel, v)))
        p += (v + capped) * t / 2
        v = capped
    stop = p + v * abs(v) / (2 * accel)
    sign = 1 if target >= stop else -1
    u, distance = sign * v, sign * (target - p)
    peak = min(speed, math.sqrt(max(0, accel * distance + u * u / 2)))
    cruise_distance = distance - (2 * peak * peak - u * u) / (2 * accel)
    phases.extend(
        [
            (max(0, (peak - u) / accel), sign * accel),
            (max(0, cruise_distance) / speed, 0),
            (peak / accel, -sign * accel),
        ]
    )
    return Axis(position, velocity, tuple(phases))


@dataclass(frozen=True)
class Leg:
    x: Axis
    y: Axis

    @classmethod
    def to(cls, start, velocity, target, *, cfg=TRAVEL_CFG):
        speed = cfg.max_speed / math.sqrt(2)
        accel = cfg.max_acceleration / math.sqrt(2)
        return cls(
            *(
                axis_to(p, v, g, speed=speed, accel=accel)
                for p, v, g in zip(start, velocity, target, strict=True)
            )
        )

    @property
    def duration(self):
        return max(self.x.duration, self.y.duration)

    def position(self, t):
        return self.x.at(t)[0], self.y.at(t)[0]

    def velocity(self, t):
        return self.x.at(t)[1], self.y.at(t)[1]


def path_samples(first, second=None):
    end = first.duration
    return [second.position(t - end) if second and t > end else first.position(t) for t in TIMES]


def inside(p):
    x, y = map(abs, p)
    return x <= END_LIMIT and y <= SIDE and (x < PENALTY_FRONT or y > PENALTY_HALF_WIDTH)


def interpolation_margin(cfg):
    return max(CFG.max_acceleration, cfg.max_acceleration) * INTERVAL**2 / 4


def clear(path, tracks, clearance, *, margin=INTERPOLATION_MARGIN):
    # Use the closest point on each relative chord. Allow for deviations from
    # linear interpolation of two acceleration-bounded paths over one interval.
    for (x, y), (nx, ny) in zip(path, path[1:]):
        # The bounding rectangle also covers a curved trajectory between samples.
        if not inside((max(abs(x), abs(nx)) + margin, max(abs(y), abs(ny)) + margin)):
            return False
        min_y = 0 if ny * y <= 0 else min(abs(y), abs(ny))
        if not inside((max(abs(x), abs(nx)) + margin, max(0, min_y - margin))):
            return False
    for other in tracks:
        for (x, y), (nx, ny), (qx, qy), (nqx, nqy) in zip(path, path[1:], other, other[1:]):
            if (
                separation(x - qx, y - qy, nx - x - nqx + qx, ny - y - nqy + qy, 1)
                < clearance + margin
            ):
                return False
    return True


def velocity_track(r, request, *, cfg=CFG):
    """Extrapolate a teammate command with the same planar acceleration limit."""
    vx, vy = r.vx, r.vy
    scale = max(1.0, math.hypot(*request) / cfg.max_speed)
    dx, dy = request[0] / scale - vx, request[1] / scale - vy
    ramp = math.hypot(dx, dy) / cfg.max_acceleration
    points = []
    for t in TIMES:
        s = min(t, ramp)
        blend = (s * s / (2 * ramp) if ramp else 0) + max(0, t - ramp)
        points.append((r.x + t * vx + blend * dx, r.y + t * vy + blend * dy))
    return points


def choose(r, target, tracks, clearance, *, cfg=TRAVEL_CFG):
    start = (r.x, r.y)
    margin = interpolation_margin(cfg)
    direct = Leg.to(start, (r.vx, r.vy), target, cfg=cfg)
    path = path_samples(direct)
    if clear(path, tracks, clearance, margin=margin):
        return direct.velocity(cfg.dt), path, "direct"
    dx, dy = target[0] - r.x, target[1] - r.y
    dist = math.hypot(dx, dy)
    best = None
    if dist > 0.02:
        ux, uy = dx / dist, dy / dist
        forward = min(0.65, 0.5 * dist)
        for offset in (0.38, -0.38, 0.72, -0.72):
            via = (r.x + forward * ux - offset * uy, r.y + forward * uy + offset * ux)
            if not inside(via):
                continue
            first = Leg.to(start, (r.vx, r.vy), via, cfg=cfg)
            second = Leg.to(via, (0, 0), target, cfg=cfg)
            duration = first.duration + second.duration
            if best and duration >= best[0]:
                continue
            path = path_samples(first, second)
            if clear(path, tracks, clearance, margin=margin):
                best = duration, first.velocity(cfg.dt), path
    if best:
        return best[1], best[2], "detour"
    return (0, 0), velocity_track(r, (0, 0), cfg=cfg), "brake"


def plan_team(
    robots,
    team,
    commands,
    targets,
    roles,
    observed,
    clearance,
    counts=None,
    fallback=None,
    modes=None,
    trace=None,
    cfg=TRAVEL_CFG,
):
    """Reserve timed motion within one team. Opponents supply position history only."""
    paths = {}
    output = dict(commands)
    for r in robots:
        if r.key.team == team:
            cmd = commands[r.key]
            paths[r.key] = velocity_track(r, world_vector(cmd.vx, cmd.vy, r.yaw), cfg=cfg)
        else:
            vx, vy = observed[r.key]
            paths[r.key] = [(r.x + t * vx, r.y + t * vy) for t in TIMES]
    movers = [r for r in robots if r.key in commands and r.key in targets and inside((r.x, r.y))]
    if fallback is not None:
        fallback.retain_movers(team, movers)
    movers.sort(key=lambda r: (TIMED_ROLES.index(roles[r.key].role), r.key.id))
    for r in movers:
        others = [p for k, p in paths.items() if k != r.key]
        if fallback is None:
            v, path, mode = choose(r, targets[r.key], others, clearance, cfg=cfg)
        else:
            v, path, mode = fallback.plan(
                r, targets[r.key], roles[r.key].role, others, clearance, cfg=cfg
            )
        vx, vy = body_vector(*v, r.yaw)
        output[r.key] = replace(output[r.key], vx=vx, vy=vy)
        paths[r.key] = path
        if trace is not None:
            memo = fallback.memory.get(r.key) if fallback is not None else None
            trace[r.key] = dict(
                mode=mode,
                target=targets[r.key],
                path=[[round(x, 4), round(y, 4)] for x, y in path],
                speed_mps=math.hypot(r.vx, r.vy),
                waypoint=memo.waypoints[0]
                if memo and memo.waypoints and mode in ("errt", "escape", "hold")
                else None,
            )
        if modes is not None:
            modes[r.key] = mode
        if counts is not None:
            counts[mode] = counts.get(mode, 0) + 1
    return output
