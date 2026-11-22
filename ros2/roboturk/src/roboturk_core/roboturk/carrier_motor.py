"""Velocity-matched roller control during a contested recovery."""

import math
from .model import Command, body_vector, wrap
from .transition_attack import unit


def track_roller(w, actor, desired, omega, p):
    b = w.ball
    x, y = body_vector(b.x - actor.x, b.y - actor.y, actor.yaw)
    vx, vy = body_vector(b.vx, b.vy, actor.yaw)
    tx, ty = body_vector(*desired, actor.yaw)
    # Match the ball first, then move the mouth ahead of it to accelerate the spring-held ball.
    ax, ay = tx - vx, ty - vy
    scale = min(p["drive"], p["step_v"] / max(0.001, math.hypot(ax, ay)))
    ux = vx + scale * ax + p["grip"] * (x - 0.115)
    uy = vy + scale * ay + p["grip"] * y
    omega = max(-p["pivot"], min(p["pivot"], omega))
    # Leave enough body velocity to track the ball during a pivot. If even straight
    # tracking saturates, use the requested fraction that requires least body speed.
    best = None
    for fraction in (1.0, 0.7, 0.35, 0.0):
        turn = omega * fraction
        cx, cy = ux + turn * y, uy - turn * x
        required = math.hypot(cx, cy)
        if best is None or required < best[0]:
            best = required, cx, cy, turn
        if required <= p["cap"]:
            best = required, cx, cy, turn
            break
    speed, cx, cy, turn = best
    rate = min(1.0, p["cap"] / max(0.001, speed))
    return Command(cx * rate, cy * rate, turn, 1, 0)


def protected_motion(w, actor, rival, mate, default, last, p, carry):
    b = w.ball
    ox, oy = body_vector(b.x - rival.x, b.y - rival.y, rival.yaw)
    rx, ry = body_vector(b.vx - rival.vx, b.vy - rival.vy, rival.yaw)
    # Project the mouth, including the marker's rotation, to choose its open edge.
    future_x = ox + 0.08 * (rx + rival.omega * oy)
    future_y = oy + 0.08 * (ry - rival.omega * ox)
    closeness = min(
        max(abs(ox - 0.115) / 0.035, abs(oy) / 0.045),
        max(abs(future_x - 0.115) / 0.035, abs(future_y) / 0.045),
    )
    away = unit(b.x - rival.x, b.y - rival.y)
    sign = 1 if 0.6 * oy + 0.4 * future_y >= 0 else -1
    lx, ly = -sign * math.sin(rival.yaw), sign * math.cos(rival.yaw)
    if future_x > 0.145 and (future_x - 0.115) / 0.035 > abs(future_y) / 0.045:
        lx, ly = math.cos(rival.yaw), math.sin(rival.yaw)
    ex, ey = unit(p["retreat"] * away[0] + lx, p["retreat"] * away[1] + ly)
    gap = math.hypot(rival.x - b.x, rival.y - b.y)
    threat = max(0.0, p["jaw_buffer"] - closeness)
    if last in ("shield", "sidestep"):
        threat += p["linger"] if gap < p["near"] else 0
    edge = abs(body_vector(b.x - actor.x, b.y - actor.y, actor.yaw)[1]) / 0.045
    mode = "release"
    omega = default.omega
    if (threat > 0 or carry) and gap < p["near"]:
        if edge > 0.67 or abs(oy) < p["side_at"]:
            mode = "shield"
            omega = 3.8 * wrap(math.atan2(away[1], away[0]) - actor.yaw)
        else:
            mode = "sidestep"
            omega = 3.8 * wrap(math.atan2(mate.y - b.y, mate.x - b.x) - actor.yaw)
    elif carry:
        mode = "sidestep"
    speed = p["speed_out"] * max(0.0, 1.0 - max(0.0, gap - 0.17) / p["fade"])
    if mode == "release" and gap >= p["near"] and math.hypot(b.vx, b.vy) < p["settled"]:
        return default, mode, edge, threat
    target = ex * speed, ey * speed
    return track_roller(w, actor, target, omega, p), mode, edge, threat
