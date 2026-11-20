"""Training-only continuous-play teachers. Never used by neural deployment."""

import math
import numpy as np
from .model import Command, Intent, Skill, body_vector, world_vector, possession, wrap
from .capture import CaptureTeacher
from .continuous import encode, ordered, SPEED


def clearance(w, a, b, keys):
    vx, vy = b[0] - a[0], b[1] - a[1]
    den = max(1e-8, vx * vx + vy * vy)
    return min(
        math.hypot(
            w.robot(k).x
            - (
                a[0]
                + np.clip(((w.robot(k).x - a[0]) * vx + (w.robot(k).y - a[1]) * vy) / den, 0, 1)
                * vx
            ),
            w.robot(k).y
            - (
                a[1]
                + np.clip(((w.robot(k).x - a[0]) * vx + (w.robot(k).y - a[1]) * vy) / den, 0, 1)
                * vy
            ),
        )
        for k in keys
    )


def avoid(w, k, c):
    """Demonstration labels only: boundary and relative closing-speed coaching."""
    r = w.robot(k)
    vx, vy = world_vector(c.vx, c.vy, r.yaw)
    for axis, value in enumerate((r.x, r.y)):
        bound = (4.2, 2.65)[axis]
        v = (vx, vy)[axis]
        limit = max(0.0, (bound - abs(value)) * 3)
        if value * v > 0 and abs(v) > limit:
            if axis == 0:
                vx = math.copysign(limit, v)
            else:
                vy = math.copysign(limit, v)
    for p in w.robots:
        if p.key == k:
            continue
        dx, dy = r.x - p.x, r.y - p.y
        d = max(1e-6, math.hypot(dx, dy))
        if d < 0.6:
            ux, uy = dx / d, dy / d
            radial = vx * ux + vy * uy
            minimum = p.vx * ux + p.vy * uy - 3 * (d - 0.23)
            if radial < minimum:
                vx += (minimum - radial) * ux
                vy += (minimum - radial) * uy
    for axis, value in enumerate((r.x, r.y)):
        bound = (4.2, 2.65)[axis]
        velocity = (vx, vy)[axis]
        limit = 3 * (bound - abs(value))
        if math.copysign(1, value) * velocity > limit:
            if axis == 0:
                vx = math.copysign(1, value) * limit
            else:
                vy = math.copysign(1, value) * limit
    scale = max(1, math.hypot(vx, vy) / SPEED)
    x, y = body_vector(vx / scale, vy / scale, r.yaw)
    return Command(x, y, c.omega, c.dribbler, c.kick)


class ContinuousTeacher:
    def actions(self, match):
        return np.stack([self.action(match, k) for k in match.memory.mode])

    def action(self, match, k):
        w, m, cfg = match.world, match.memory, match.cfg
        r, b = w.robot(k), w.ball
        sign = w.direction(k.team)
        keys = ordered(w, k)
        mate = w.robot(keys[1])
        opps = [w.robot(key) for key in keys[2:]]
        held = possession(w, k, cfg)
        mateheld = possession(w, mate.key, cfg)
        enemy = next((p for p in opps if possession(w, p.key, cfg)), None)
        ownrange = math.hypot(b.x - r.x, b.y - r.y)
        materange = math.hypot(b.x - mate.x, b.y - mate.y)
        # A local hysteresis test admitted two simultaneous cover assignments.
        # The training teacher assigns exactly one nearest teammate to chase.
        chaser = ownrange < materange
        if abs(ownrange - materange) < 1e-9:
            chaser = k.id == 0
        closest = min(opps, key=lambda p: math.hypot(p.x - r.x, p.y - r.y))
        pressure = math.hypot(closest.x - r.x, closest.y - r.y)
        gy = 0.25 if b.y >= 0 else -0.25
        if held:
            contested = any(possession(w, p.key, cfg) for p in opps)
            distance = math.hypot(sign * 4.5 - b.x, gy - b.y)
            can_pass = (
                (sign * (mate.x - r.x) > 0.35 or contested)
                and math.hypot(mate.x - r.x, mate.y - r.y) > 0.75
                and (
                    clearance(w, (b.x, b.y), (mate.x, mate.y), keys[2:]) > 0.20
                    or contested
                    or m.mode[k] == 2
                )
                and pressure < 1.6
                and w.time - m.last_kick[k] > 1
            )
            if (
                (distance < 1.8 or (m.mode[k] == 3 and distance < 2.1))
                and sign * b.x < 4.15
                and abs(b.y) < 1.4
            ):
                mode = 3
                tx, ty = sign * 4.5, gy
                speed = 0.0
            elif can_pass:
                mode = 2
                tx, ty = mate.x, mate.y
                speed = 0.0
            else:
                mode = 1
                tx, ty = sign * 3.2, 0.0
                # Side-step a blocking robot during demonstrations; deployment learns movement.
                if (
                    sign * (closest.x - r.x) > 0.0
                    and sign * (closest.x - r.x) < 1.5
                    and abs(closest.y - r.y) < 0.6
                ):
                    ty = float(
                        np.clip(closest.y + (1 if r.y >= closest.y else -1) * 0.85, -2.1, 2.1)
                    )
                    tx = sign * min(3.1, sign * closest.x + 0.7)
                speed = 0.6
            angle = wrap(math.atan2((ty - b.y) * (0.8 if mode == 3 else 1), tx - b.x) - r.yaw)
            omega = float(np.clip(2.2 * angle, -1.1, 1.1))
            bx, by = body_vector(b.x - r.x, b.y - r.y, r.yaw)
            vx = speed * max(0, math.cos(angle)) ** 3 + 3 * (bx - cfg.possession_x)
            vy = -cfg.possession_x * omega + 3 * by
            c = Command(vx, vy, omega, 1)
            if mode in (2, 3) and abs(angle) < 0.035 and (m.dwell[k] >= 20 or contested):
                c = Command(kick=2.2 if mode == 2 else 3.5)
        elif mateheld:
            mode = 6
            tx = float(np.clip(b.x + sign * 1.15, -3.4, 3.4))
            ty = float(np.clip(b.y + (-1 if b.y >= 0 else 1) * 1.35, -2.25, 2.25))
            c = self.goto(w, k, (tx, ty), face=(b.x, b.y))
        elif chaser:
            mode = 4 if enemy else 0
            c = CaptureTeacher().command(w, k, Intent(Skill.CHASE, (b.x, b.y)), cfg)
            # After a kick, another teammate receives. This is teacher memory only.
            if w.time - m.last_kick[k] < 0.9:
                mode = 6
                c = self.goto(w, k, (r.x - sign * 0.2, r.y), face=(b.x, b.y))
        else:
            mode = 5
            gx = -sign * 4.1
            tx = gx + 0.38 * (b.x - gx)
            ty = 0.38 * b.y
            c = self.goto(w, k, (tx, ty), face=(b.x, b.y))
        return encode(mode, avoid(w, k, c))

    @staticmethod
    def goto(w, k, target, face):
        r = w.robot(k)
        vx, vy = 2.8 * (target[0] - r.x), 2.8 * (target[1] - r.y)
        scale = max(1, math.hypot(vx, vy) / 0.9)
        x, y = body_vector(vx / scale, vy / scale, r.yaw)
        omega = float(np.clip(4 * wrap(math.atan2(face[1] - r.y, face[0] - r.x) - r.yaw), -4, 4))
        return Command(x, y, omega, 1)
