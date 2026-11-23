"""Predict robot encounters during a ten-player restart, in field coordinates."""

from dataclasses import replace
import math

from .model import body_vector, world_vector


def separation(dx, dy, vx, vy, horizon):
    """Closest center distance during constant relative translation."""
    t = max(0.0, min(horizon, -(dx * vx + dy * vy) / max(1e-9, vx * vx + vy * vy)))
    return math.hypot(dx + t * vx, dy + t * vy)


def route(w, commands, parameters):
    horizon, margin, angle = parameters
    velocities = {
        r.key: world_vector(commands[r.key].vx, commands[r.key].vy, r.yaw) for r in w.robots
    }
    result = commands.copy()
    for r in w.robots:
        vx, vy = velocities[r.key]
        speed = math.hypot(vx, vy)
        if speed < 0.035:
            continue
        nearby = []
        for q in w.robots:
            if q.key == r.key or math.hypot(r.x - q.x, r.y - q.y) > margin + horizon * (speed + 2):
                continue
            qx, qy = velocities[q.key]
            nearby.append((r.x - q.x, r.y - q.y, qx, qy, q.vx, q.vy))

        def clearance(x, y):
            return min(
                (
                    min(
                        separation(dx, dy, x - qx, y - qy, horizon),
                        separation(dx, dy, x - mx, y - my, min(horizon, 0.24)),
                    )
                    for dx, dy, qx, qy, mx, my in nearby
                ),
                default=math.inf,
            )

        if clearance(vx, vy) >= margin:
            continue
        # Prefer the smallest change of velocity that provides clearance.
        # Check the field apron too, since retrieval goes beyond the white line.
        best = None
        for scale in (1, 0.65, 0.32, 0):
            for turn in (0, angle, -angle, 2 * angle, -2 * angle, 1.5, -1.5) if scale else (0,):
                x = scale * (vx * math.cos(turn) - vy * math.sin(turn))
                y = scale * (vx * math.sin(turn) + vy * math.cos(turn))
                gap = clearance(x, y)
                edge = max(0, abs(r.x + x * horizon) - 4.84, abs(r.y + y * horizon) - 3.35)
                cost = (
                    (x - vx) ** 2 + (y - vy) ** 2 + 320 * max(0, margin - gap) ** 2 + 150 * edge**2
                )
                # Favor opening a gap when the robots are already too close.
                if nearby and gap < margin:
                    end = min(
                        math.hypot(dx + (x - qx) * 0.16, dy + (y - qy) * 0.16)
                        for dx, dy, qx, qy, _, _ in nearby
                    )
                    cost += 150 * max(0, margin - end) ** 2
                if best is None or cost < best[0]:
                    best = cost, x, y
        _, x, y = best
        velocities[r.key] = (x, y)
        bx, by = body_vector(x, y, r.yaw)
        result[r.key] = replace(commands[r.key], vx=bx, vy=by)
    return result
