"""Choose a moving shield direction from predicted opponent mouth positions."""

from dataclasses import replace
import math
from .blocked_outlet import finish_motion
from .carrier_motor import track_roller
from .model import wrap
from .transition_attack import unit


def sprint(w, r, p):
    b = w.ball
    rivals = [q for q in w.robots if q.key.team != r.key.team]
    direction = w.direction(r.key.team)
    nearest = min(rivals, key=lambda q: math.hypot(q.x - b.x, q.y - b.y))
    gap = math.hypot(nearest.x - b.x, nearest.y - b.y)
    best = None
    for j in range(20):
        a = j * math.pi / 10
        ux, uy = math.cos(a), math.sin(a)
        clear = 2.0
        for dt in (0.14, 0.38, 0.72):
            x = b.x + (0.20 * b.vx + p["speed"] * ux) * dt
            y = b.y + (0.20 * b.vy + p["speed"] * uy) * dt
            margin = min(4.34 - abs(x), 2.88 - abs(y))
            clear = min(clear, max(-1.0, 2.0 * margin))
            for q in rivals:
                mx = q.x + min(dt, 0.35) * q.vx + 0.115 * math.cos(q.yaw)
                my = q.y + min(dt, 0.35) * q.vy + 0.115 * math.sin(q.yaw)
                clear = min(clear, math.hypot(x - mx, y - my) / (0.5 + dt))
        forward = direction * ux - 0.22 * uy * b.y
        value = 4.4 * min(0.70, clear) + p["advance"] * forward
        value += 0.17 * (ux * b.vx + uy * b.vy)
        if best is None or value > best[0]:
            best = value, ux, uy
    _, ux, uy = best
    angle, _, aligned = finish_motion(w, r, dict(shot_clear=p["shot_clear"]), (0, 0))
    if gap < p["shield"] and not aligned:
        ex, ey = unit(b.x - nearest.x, b.y - nearest.y)
        angle = math.atan2(0.55 * uy + ey, 0.55 * ux + ex)
    cmd = track_roller(
        w,
        r,
        (p["speed"] * ux, p["speed"] * uy),
        9.0 * wrap(angle - r.yaw),
        dict(drive=0.76, step_v=0.54, grip=24, pivot=7.5, cap=1.92),
    )
    gx = 4.53 * direction
    inside = abs(b.y + (gx - b.x) * math.tan(r.yaw)) < p["margin"]
    return replace(cmd, kick=4.0 if aligned and inside else 0.0)
