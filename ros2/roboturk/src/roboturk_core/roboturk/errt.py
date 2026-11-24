"""Execution-extended RRT waypoint search for otherwise blocked off-ball moves."""

from dataclasses import dataclass
import math
import random

from .five_roles import CFG
from .timed_motion import END_LIMIT, SIDE, Leg, clear, inside, path_samples

EXTENSIONS = 96
STEP = 0.44


@dataclass
class SearchMemory:
    rng: random.Random
    role: str
    goal: tuple
    waypoints: tuple = ()
    cost_bias: float = 0.12


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


class ERRTFallback:
    def __init__(self, seed):
        self.seed = seed
        self.memory = {}

    def search(self, r, target, role, tracks, clearance):
        start = (r.x, r.y)
        disks = [p[0] for p in tracks]
        if not inside(target) or any(math.dist(start, p) < clearance for p in disks):
            return None
        key = r.key
        memo = self.memory.get(key)
        if memo is None or memo.role != role or math.dist(memo.goal, target) > 0.50:
            rng = random.Random(self.seed * 41 + 17 * (key.team == "yellow") + key.id)
            memo = self.memory[key] = SearchMemory(rng, role, target)
        memo.goal = target

        def execute(points):
            # Connect to the furthest usable waypoint. The speed profile can curve
            # away from the chord when the incoming velocity is nonzero.
            for i in reversed(range(len(points))):
                point = points[i]
                if math.dist(start, point) < 0.07 or not edge_clear(start, point, disks, clearance):
                    continue
                leg = Leg.to(start, (r.vx, r.vy), point)
                path = path_samples(leg)
                if clear(path, tracks, clearance):
                    memo.waypoints = tuple(points[i:])
                    return leg.velocity(CFG.dt), path
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
