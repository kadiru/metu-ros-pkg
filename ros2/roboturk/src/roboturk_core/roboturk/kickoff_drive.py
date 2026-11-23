"""Move persistent robot IDs to a seeded formation during an awarded kickoff."""

import heapq
import math

from .model import Command, body_vector, wrap

CLEARANCE = 0.30
SPEED = 1.0
TOLERANCE = 0.004


def clear(start, end, obstacles):
    """A swept center must clear parked robots, or move apart if initially close."""
    x, y = start
    dx, dy = end[0] - x, end[1] - y
    norm = max(1e-12, dx * dx + dy * dy)
    for ox, oy in obstacles:
        t = max(0, min(1, ((ox - x) * dx + (oy - y) * dy) / norm))
        gap = math.hypot(x + t * dx - ox, y + t * dy - oy)
        if gap + 1e-7 < min(CLEARANCE, math.hypot(x - ox, y - oy)):
            return False
    return True


def path_to(start, goal, obstacles):
    if any(math.dist(goal, o) < CLEARANCE for o in obstacles):
        return []
    if clear(start, goal, obstacles):
        return [goal]
    # A short grid search is needed when a straight approach is blocked.
    queue = [(math.dist(start, goal), 0.0, start)]
    parent = {start: None}
    costs = {start: 0.0}
    while queue:
        _, cost, here = heapq.heappop(queue)
        if cost > costs[here]:
            continue
        if clear(here, goal, obstacles):
            route = [goal]
            while here != start:
                route.append(here)
                here = parent[here]
            return list(reversed(route))
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)):
            nxt = round(here[0] + 0.24 * dx, 5), round(here[1] + 0.24 * dy, 5)
            distance = cost + math.dist(here, nxt)
            if abs(nxt[0]) > 4.68 or abs(nxt[1]) > 3.13 or distance >= costs.get(nxt, math.inf):
                continue
            if not clear(here, nxt, obstacles):
                continue
            costs[nxt], parent[nxt] = distance, here
            heapq.heappush(queue, (distance + math.dist(nxt, goal), distance, nxt))
    return []


class KickoffDrive:
    def __init__(self, targets):
        self.targets = {r.key: r for r in targets.robots}
        self.paths = {}

    def _plan(self, robots):
        positions = {r.key: (r.x, r.y) for r in robots}
        pending = [
            k
            for k, p in positions.items()
            if math.dist(p, (self.targets[k].x, self.targets[k].y)) > TOLERANCE
        ]
        # Reserve disjoint straight routes for simultaneous travel. A parked robot
        # stays an obstacle until the current group arrives at its waypoints.
        reserved = []
        for key in pending:
            start = positions[key]
            g = self.targets[key]
            goal = g.x, g.y
            obstacles = [p for k, p in positions.items() if k != key] + reserved
            if not clear(start, goal, obstacles) or any(
                math.dist(goal, o) < CLEARANCE for o in obstacles
            ):
                continue
            self.paths[key] = [goal]
            n = math.ceil(math.dist(start, goal) / 0.08)
            reserved.extend(
                (start[0] + (goal[0] - start[0]) * i / n, start[1] + (goal[1] - start[1]) * i / n)
                for i in range(n + 1)
            )
        if self.paths or not pending:
            return
        for key in pending:
            g = self.targets[key]
            path = path_to(
                positions[key], (g.x, g.y), [p for k, p in positions.items() if k != key]
            )
            if path:
                self.paths[key] = path
                return
        # A cycle of occupied destinations needs a vacant staging position.
        for key in pending:
            start = positions[key]
            obstacles = [p for k, p in positions.items() if k != key]
            for y in (2.72, -2.72):
                for x in (-2, 0, 2):
                    if math.dist(start, (x, y)) < CLEARANCE:
                        continue
                    path = path_to(start, (x, y), obstacles)
                    if path:
                        self.paths[key] = path
                        return
        raise RuntimeError("No clear route to the kickoff formation")

    def commands(self, world, dt):
        if not self.paths:
            self._plan(world.robots)
        out = {}
        ready = True
        for r in world.robots:
            target = self.targets[r.key]
            path = self.paths.get(r.key)
            dx = dy = 0.0
            if path:
                dx, dy = path[0][0] - r.x, path[0][1] - r.y
                if math.hypot(dx, dy) <= TOLERANCE:
                    path.pop(0)
                    if not path:
                        del self.paths[r.key]
                    dx = dy = 0.0
            scale = max(dt, math.hypot(dx, dy) / SPEED)
            vx, vy = body_vector(dx / scale, dy / scale, r.yaw)
            angle = wrap(target.yaw - r.yaw)
            out[r.key] = Command(vx=vx, vy=vy, omega=max(-2.0, min(2.0, angle / dt)))
            ready &= (
                math.hypot(r.x - target.x, r.y - target.y) <= TOLERANCE
                and abs(angle) < 0.015
                and math.hypot(r.vx, r.vy) < 0.035
                and abs(r.omega) < 0.08
            )
        return out, ready
