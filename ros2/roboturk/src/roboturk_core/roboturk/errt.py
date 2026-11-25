"""Execution-extended RRT waypoint search for otherwise blocked off-ball moves."""

from dataclasses import dataclass
import math
import random

from .five_roles import CFG
from .placement_traffic import separation
from .timed_motion import (
    END_LIMIT,
    INTERPOLATION_MARGIN,
    SIDE,
    TRAVEL_CFG,
    Leg,
    choose,
    clear,
    inside,
    interpolation_margin,
    path_samples,
)

EXTENSIONS = 96
STEP = 0.44
DIRECT_CLEAR_TICKS = 4
DIRECT_TIME_SAVING = 0.20
DIRECT_EXTRA_CLEARANCE = 0.02


@dataclass
class SearchMemory:
    rng: random.Random
    role: str
    goal: tuple
    waypoints: tuple = ()
    cost_bias: float = 0.12
    committed: tuple | None = None
    restoring: bool = False
    entry: tuple | None = None
    interrupted: bool = False
    direct_clear_ticks: int = 0


def edge_clear(a, b, disks, radius):
    """Conservative static geometry for proposing a route. Execution is checked in time."""
    x, y = a
    dx, dy = b[0] - x, b[1] - y
    far_x = max(abs(x), abs(b[0]))
    near_y = min(abs(y), abs(b[1])) if y * b[1] > 0 else 0
    if not inside(a) or not inside(b) or not inside((far_x, near_y)):
        return False
    length_sq = dx * dx + dy * dy
    for ox, oy in disks:
        u = max(0, min(1, ((ox - x) * dx + (oy - y) * dy) / length_sq)) if length_sq else 0
        if (x + u * dx - ox) ** 2 + (y + u * dy - oy) ** 2 < radius * radius:
            return False
    return True


def opens_clearance(path, tracks, clearance, *, margin=INTERPOLATION_MARGIN):
    """Recover planning clearance without moving closer to any encroaching robot."""
    clear_tracks = []
    for track in tracks:
        initial = math.dist(path[0], track[0])
        if initial >= clearance + margin:
            clear_tracks.append(track)
            continue
        if initial <= 2 * CFG.robot_radius + margin:
            return False
        # Relative distance must grow along each sampled chord until we have
        # restored the full planning buffer. Account for interpolation error.
        recovered = False
        for a, b, p, q in zip(path, path[1:], track, track[1:]):
            x, y = a[0] - p[0], a[1] - p[1]
            dx, dy = b[0] - q[0] - x, b[1] - q[1] - y
            if recovered:
                if separation(x, y, dx, dy, 1) < clearance + margin:
                    return False
            elif x * dx + y * dy < -1e-10:
                return False
            recovered = math.hypot(x + dx, y + dy) >= clearance + margin
        if not recovered:
            return False
    return clear(path, clear_tracks, clearance, margin=margin)


def checked_leg(r, point, tracks, clearance, restoring=False, *, cfg=TRAVEL_CFG):
    leg = Leg.to((r.x, r.y), (r.vx, r.vy), point, cfg=cfg)
    path = path_samples(leg)
    admissible = opens_clearance if restoring else clear
    if admissible(path, tracks, clearance, margin=interpolation_margin(cfg)):
        return leg.velocity(cfg.dt), path
    return None


class ERRTFallback:
    def __init__(self, seed, commit_waypoints=False, early_release=False):
        self.seed = seed
        self.commit_waypoints = commit_waypoints
        self.early_release = early_release
        self.memory = {}

    def retain_movers(self, team, movers):
        if self.commit_waypoints:
            eligible = {r.key for r in movers}
            for key in list(self.memory):
                if key.team == team and key not in eligible:
                    del self.memory[key]

    def plan(self, r, target, role, tracks, clearance, *, cfg=TRAVEL_CFG):
        if self.commit_waypoints:
            held = self.resume(r, target, role, tracks, clearance, cfg=cfg)
            if held is not None:
                if self.early_release:
                    shortcut = self.release_to_direct(r, target, tracks, clearance, cfg=cfg)
                    if shortcut is not None:
                        return *shortcut, "shortcut"
                return *held, "hold"
        velocity, path, mode = choose(r, target, tracks, clearance, cfg=cfg)
        if mode != "brake":
            if self.commit_waypoints and r.key in self.memory:
                self.memory[r.key].interrupted = True
            return velocity, path, mode
        motion = self.search(r, target, role, tracks, clearance, cfg=cfg)
        if motion is None:
            return velocity, path, mode
        return *motion, ("escape" if self.memory[r.key].restoring else "errt")

    def release_to_direct(self, r, target, tracks, clearance, *, cfg=TRAVEL_CFG):
        memo = self.memory[r.key]
        if memo.restoring:
            return None
        start, velocity = (r.x, r.y), (r.vx, r.vy)
        waypoint = memo.committed
        direct = Leg.to(start, velocity, target, cfg=cfg)
        via_waypoint = Leg.to(start, velocity, waypoint, cfg=cfg).duration
        via_waypoint += Leg.to(waypoint, (0, 0), target, cfg=cfg).duration
        path = path_samples(direct)
        if direct.duration + DIRECT_TIME_SAVING > via_waypoint or not clear(
            path, tracks, clearance + DIRECT_EXTRA_CLEARANCE, margin=interpolation_margin(cfg)
        ):
            memo.direct_clear_ticks = 0
            return None
        memo.direct_clear_ticks += 1
        if memo.direct_clear_ticks < DIRECT_CLEAR_TICKS:
            return None
        memo.committed = None
        memo.direct_clear_ticks = 0
        memo.interrupted = True
        return direct.velocity(cfg.dt), path

    def resume(self, r, target, role, tracks, clearance, *, cfg=TRAVEL_CFG):
        memo = self.memory.get(r.key)
        if memo is None:
            return None
        if memo.role != role or math.dist(memo.goal, target) > 0.50:
            del self.memory[r.key]
            return None
        point = memo.committed
        if point is None:
            return None
        if math.hypot(r.x - point[0], r.y - point[1]) >= 0.07:
            motion = checked_leg(r, point, tracks, clearance, memo.restoring, cfg=cfg)
            if motion is not None:
                return motion
        memo.committed = None
        memo.restoring = False
        memo.direct_clear_ticks = 0
        return None

    def search(self, r, target, role, tracks, clearance, *, cfg=TRAVEL_CFG):
        start = (r.x, r.y)
        disks = [p[0] for p in tracks]
        if not inside(target):
            return None
        key = r.key
        memo = self.memory.get(key)
        if memo is None or memo.role != role or math.dist(memo.goal, target) > 0.50:
            rng = random.Random(self.seed * 41 + 17 * (key.team == "yellow") + key.id)
            memo = self.memory[key] = SearchMemory(rng, role, target)
        memo.goal = target

        threshold = clearance + interpolation_margin(cfg) if self.commit_waypoints else clearance
        if any(math.dist(start, p) < threshold for p in disks):
            if not self.commit_waypoints or math.hypot(r.vx, r.vy) > 0.08:
                return None
            # A brake can leave a stopped runner inside the nominal buffer.
            # Try one outward tree extension, favoring a clear onward route.
            ox, oy = min(disks, key=lambda p: math.dist(start, p))
            away = math.atan2(r.y - oy, r.x - ox)
            candidates = []
            for offset in (0, 45, -45, 70, -70, 90, -90):
                angle = away + math.radians(offset)
                point = (r.x + STEP * math.cos(angle), r.y + STEP * math.sin(angle))
                blocked = not edge_clear(point, target, disks, clearance)
                candidates.append((blocked, math.dist(point, target), point))
            for _, _, point in sorted(candidates):
                motion = checked_leg(r, point, tracks, clearance, restoring=True, cfg=cfg)
                if motion is not None:
                    memo.committed = point
                    memo.waypoints = (point,)
                    memo.restoring = True
                    return motion
            return None

        def execute(points):
            # Connect to the furthest usable waypoint. The speed profile can curve
            # away from the chord when the incoming velocity is nonzero.
            for i in reversed(range(len(points))):
                point = points[i]
                if math.dist(start, point) < 0.07 or not edge_clear(start, point, disks, clearance):
                    continue
                motion = checked_leg(r, point, tracks, clearance, cfg=cfg)
                if motion is not None:
                    memo.waypoints = tuple(points[i:])
                    if self.commit_waypoints:
                        returning = (
                            memo.interrupted
                            and memo.entry is not None
                            and math.dist(start, memo.entry) < 0.07
                        )
                        if self.commit_waypoints != "on-conflict" or returning:
                            memo.committed = point
                        if memo.entry is None or memo.interrupted:
                            memo.entry = start
                        memo.interrupted = False
                    return motion
            return None

        cached = execute(memo.waypoints)
        if cached:
            return cached
        nodes = [start]
        parents = [-1]
        lengths = [0.0]
        rng = memo.rng
        for _ in range(EXTENSIONS):
            draw = rng.random()
            if draw < 0.12:
                sample = target
            elif draw < 0.45 and memo.waypoints:
                sample = rng.choice(memo.waypoints)
            elif draw > 0.9:
                sample = (rng.uniform(-END_LIMIT, END_LIMIT), rng.uniform(-SIDE, SIDE))
            else:
                sample = (
                    rng.uniform(min(start[0], target[0]) - 1.1, max(start[0], target[0]) + 1.1),
                    rng.uniform(min(start[1], target[1]) - 1.1, max(start[1], target[1]) + 1.1),
                )
            parent = min(
                range(len(nodes)),
                key=lambda i: math.dist(nodes[i], sample) + memo.cost_bias * lengths[i],
            )
            a = nodes[parent]
            distance = math.dist(a, sample)
            if distance < 0.04:
                continue
            fraction = min(1, STEP / distance)
            b = (a[0] + fraction * (sample[0] - a[0]), a[1] + fraction * (sample[1] - a[1]))
            if not edge_clear(a, b, disks, clearance + 0.006):
                continue
            nodes.append(b)
            parents.append(parent)
            lengths.append(lengths[parent] + min(STEP, distance))
            if not edge_clear(b, target, disks, clearance + 0.006):
                continue
            index = len(nodes) - 1
            chain = []
            while index:
                chain.append(nodes[index])
                index = parents[index]
            result = execute(chain[::-1])
            if result:
                memo.cost_bias = min(0.30, memo.cost_bias + 0.025)
                return result
        # Encourage exploration after a failed search and cheaper paths after success.
        memo.cost_bias = max(0, memo.cost_bias - 0.05)
        return None
