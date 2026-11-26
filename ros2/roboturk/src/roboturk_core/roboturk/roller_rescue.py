"""Correct an impending loss of the ball without interrupting an otherwise stable carry."""

import math

from .carrier_motor import track_roller
from .model import body_vector, world_vector
from .transition_attack import unit


def rescue_motion(w, actor, rival, default, p, sole_age):
    b = w.ball
    x, y = body_vector(b.x - actor.x, b.y - actor.y, actor.yaw)
    vx, vy = body_vector(b.vx, b.vy, actor.yaw)
    # Evaluate the *requested* roller velocity. The previous body velocity can point
    # in the opposite direction immediately after the strategy switches to a pass.
    mx, my = default.vx - default.omega * y, default.vy + default.omega * x
    look = p["rescue_look"]
    px = x - 0.115 + look * (vx - mx)
    py = y + look * (vy - my)
    edge = max(abs(px) / 0.035, abs(py) / 0.045)
    ox, oy = body_vector(b.x - rival.x, b.y - rival.y, rival.yaw)
    ex, ey = body_vector(b.vx - rival.vx, b.vy - rival.vy, rival.yaw)
    ex += rival.omega * oy
    ey -= rival.omega * ox
    incoming = min(
        max(abs(ox + t * ex - 0.115) / 0.035, abs(oy + t * ey) / 0.045) for t in (0, 0.04, 0.08)
    )
    if sole_age < p["rescue_age"] or (edge < p["rescue_edge"] and incoming > p["retouch_gate"]):
        return None
    target_x, target_y = world_vector(mx, my, actor.yaw)
    mode = "shield"
    if incoming < p["retouch_gate"] and sole_age >= p["guard_age"]:
        nx, ny = unit(actor.x - rival.x, actor.y - rival.y)
        tx, ty = -ny, nx
        s = math.copysign(1, (b.x - actor.x) * tx + (b.y - actor.y) * ty or 1)
        dx, dy = unit(s * tx + 0.45 * nx, s * ty + 0.45 * ny)
        q = p["rescue_side"]
        target_x = (1 - q) * target_x + q * 1.17 * dx
        target_y = (1 - q) * target_y + q * 1.17 * dy
        mode = "sidestep"
    motor = p | dict(
        drive=p["rescue_drive"],
        step_v=p["rescue_step"],
        grip=p["rescue_grip"],
        pivot=p["rescue_turn"],
    )
    cmd = track_roller(w, actor, (target_x, target_y), default.omega, p=motor)
    return cmd, mode, edge, incoming
