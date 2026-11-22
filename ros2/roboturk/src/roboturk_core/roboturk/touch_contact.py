"""First-touch motion selection near two contacting robot bodies in the dribbler model."""

import math
import numpy as np

from .carrier_motor import track_roller
from .model import wrap
from .transition_attack import unit


def forecast(world, actor, rival, commands, p):
    """Forecast a short contact interval with the observed rival velocity and body geometry."""
    n = len(commands)
    xyz = np.tile(np.array([actor.x, actor.y], dtype=float), (n, 1))
    yaw = np.full(n, actor.yaw, dtype=float)
    ball = np.tile(np.array([world.ball.x, world.ball.y], dtype=float), (n, 1))
    v = np.tile(np.array([world.ball.vx, world.ball.vy], dtype=float), (n, 1))
    enemy = np.tile(np.array([rival.x, rival.y], dtype=float), (n, 1))
    other_yaw = rival.yaw
    local = np.array([(c.vx, c.vy) for c in commands])
    turns = np.array([c.omega for c in commands])
    other_v = np.array([rival.vx, rival.vy])
    own_risk = np.zeros(n)
    opponent_risk = np.zeros(n)
    dt = 0.01

    def norm(q):
        return np.hypot(q[:, 0], q[:, 1])

    for _ in range(p["contact_steps"]):
        co, si = np.cos(yaw), np.sin(yaw)
        velocity = np.column_stack(
            (co * local[:, 0] - si * local[:, 1], si * local[:, 0] + co * local[:, 1])
        )
        delta = xyz - enemy
        dist = norm(delta)
        axis = delta / np.maximum(0.001, dist)[:, None]
        mismatch = ((other_v - velocity) * axis).sum(1) - np.maximum(0.0, (dist - 0.195) * 20)
        correction = (0.5 * np.maximum(0.0, mismatch) * (dist < 0.33))[:, None] * axis
        velocity += correction
        rival_v = other_v - correction
        factor = (1.2 / np.maximum(1.2, np.maximum(norm(velocity), norm(rival_v))))[:, None]
        velocity *= factor
        rival_v *= factor
        mouth = xyz + 0.115 * np.column_stack((co, si))
        force = 8 * (mouth - ball) + 0.15 * (velocity - v)
        v += dt * force / (0.046 * np.maximum(1.0, norm(force) / 0.4)[:, None])
        ball += dt * v
        xyz += dt * velocity
        enemy += dt * rival_v
        yaw += dt * turns
        other_yaw += dt * rival.omega
        # Both cylinders can eject a wedged ball sideways during the recovery.
        for center, body_v in ((enemy, rival_v), (xyz, velocity)):
            delta = ball - center
            gap = norm(delta)
            normal = delta / np.maximum(0.001, gap)[:, None]
            penetration = np.maximum(0.0, p["body_contact"] - gap)
            ball += penetration[:, None] * normal
            approach = np.maximum(0.0, ((body_v - v) * normal).sum(1))
            v += ((gap < p["body_contact"]) * approach * p["impulse"])[:, None] * normal
        dx, dy = (ball - xyz).T
        c, s = np.cos(yaw), np.sin(yaw)
        error = np.maximum(np.abs(c * dx + s * dy - 0.115) / 0.035, np.abs(c * dy - s * dx) / 0.045)
        dx, dy = (ball - enemy).T
        c, s = math.cos(other_yaw), math.sin(other_yaw)
        other = np.maximum(np.abs(c * dx + s * dy - 0.115) / 0.035, np.abs(c * dy - s * dx) / 0.045)
        own_risk = np.maximum(own_risk, np.maximum(0.0, error - p["own_edge"]))
        opponent_risk = np.maximum(opponent_risk, np.maximum(0.0, p["rival_edge"] - other))
    return own_risk**2 + p["retouch_weight"] * opponent_risk**2


def protect_contact(world, actor, rival, default, p):
    b = world.ball
    if math.hypot(rival.x - b.x, rival.y - b.y) > p["contact_gate"]:
        return None
    nx, ny = unit(actor.x - rival.x, actor.y - rival.y)
    tx, ty = -ny, nx
    gap = (b.x - rival.x) * tx + (b.y - rival.y) * ty
    along = (b.vx - rival.vx) * tx + (b.vy - rival.vy) * ty
    sign = math.copysign(1, gap + 0.035 * along or 1)
    tx, ty = sign * tx, sign * ty
    away = math.atan2(b.y - rival.y, b.x - rival.x)
    clearance = p["lateral_speed"]
    commands = [default]
    for lateral, retreat in [(1, 0.1), (0.7, 0.65), (0.2, 1), (-0.5, 0.65)]:
        ux, uy = unit(lateral * tx + retreat * nx, lateral * ty + retreat * ny)
        desired = clearance * ux, clearance * uy
        for heading in (math.atan2(uy, ux), away, actor.yaw):
            turn = 5.5 * wrap(heading - actor.yaw)
            motor = p | dict(drive=p["direct_drive"], step_v=p["direct_step"])
            commands.append(track_roller(world, actor, desired, turn, motor))
    risk = forecast(world, actor, rival, commands, p)
    best = int(np.argmin(risk))
    if best and risk[best] + p["change_margin"] < risk[0]:
        return commands[best], float(risk[0] - risk[best])
    return None


def swing_roller(world, actor, rival, p):
    """Carry around the outside of the two chassis until the rival is behind the mouth."""
    b = world.ball
    nx, ny = unit(rival.x - actor.x, rival.y - actor.y)
    tx, ty = -ny, nx
    width = (b.x - actor.x) * tx + (b.y - actor.y) * ty
    forward = (b.x - actor.x) * nx + (b.y - actor.y) * ny
    gap = math.hypot(b.x - rival.x, b.y - rival.y)
    if gap > p["swing_range"] or forward < p["swing_clear"]:
        return None
    flow = (b.vx - 0.5 * (actor.vx + rival.vx)) * tx + (b.vy - 0.5 * (actor.vy + rival.vy)) * ty
    sign = math.copysign(1, width + p["swing_look"] * flow or 1)
    tx, ty = sign * tx, sign * ty
    hx, hy = unit(tx - p["back_angle"] * nx, ty - p["back_angle"] * ny)
    target_yaw = math.atan2(hy, hx)
    omega = p["swing_turn"] * wrap(target_yaw - actor.yaw)
    turn = max(-p["swing_pivot"], min(p["swing_pivot"], omega))
    dx, dy = unit(tx - p["pull_back"] * nx, ty - p["pull_back"] * ny)
    vx, vy = p["swing_speed"] * dx, p["swing_speed"] * dy
    # Move the ball tangentially with the mouth as well as translating clear.
    vx -= p["arc_fraction"] * turn * (b.y - actor.y)
    vy += p["arc_fraction"] * turn * (b.x - actor.x)
    motor = p | dict(drive=p["swing_drive"], step_v=p["swing_step"], pivot=p["swing_pivot"])
    cmd = track_roller(world, actor, (vx, vy), turn, motor)
    return cmd, gap
