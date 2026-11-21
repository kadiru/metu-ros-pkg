"""Select a reachable outlet ahead of a teammate while the carrier pivots."""

from dataclasses import replace
import math
import numpy as np
from .model import wrap
from .three_recipient import candidates, choose


def route(w, actor, weights):
    base = choose(w, actor, weights)
    enemy = [r for r in w.robots if r.key.team != actor.key.team]
    direction = w.direction(actor.key.team)
    best = None
    for j, m in enumerate(candidates(w, actor)):
        for dx, dy in (
            (0, 0),
            (0.5, 0),
            (0.25, 0.52),
            (0.25, -0.52),
            (-0.25, 0.42),
            (-0.25, -0.42),
        ):
            x = float(np.clip(m.x + dx * direction, -4.03, 4.03))
            y = float(np.clip(m.y + dy, -2.56, 2.56))
            bx, by = x - w.ball.x, y - w.ball.y
            dist = max(0.01, math.hypot(bx, by))
            ux, uy = bx / dist, by / dist
            turn = abs(wrap(math.atan2(by, bx) - actor.yaw))
            # Clearance at the expected encounter includes limited opponent
            # motion during aiming and ball flight. An opponent behind the
            # kicking plane cannot intercept just by being close to the ball.
            gap = 2.0
            for r in enemy:
                rx, ry = r.x - w.ball.x, r.y - w.ball.y
                along = rx * ux + ry * uy
                if along < -0.015 or along > dist + 0.16:
                    continue
                horizon = min(0.45, turn / 3 + max(0, along) / 2.2)
                px, py = rx + horizon * r.vx, ry + horizon * r.vy
                if px * ux + py * uy < -0.03:
                    continue
                gap = min(gap, abs(px * uy - py * ux), abs(rx * uy - ry * ux) + 0.12)
            move = math.hypot(x - m.x, y - m.y)
            free = min(math.hypot(r.x - x, r.y - y) for r in enemy)
            quality = (
                2.2 * min(0.55, gap)
                + 0.44 * min(1.2, free)
                + 0.18 * (x - w.ball.x) * direction
                - 0.18 * abs(dist - 1.7)
                - 0.22 * turn
                - 0.23 * move
                + 0.07 * (np.array(base.features[j]) @ weights)
            )
            data = [
                min(1, gap / 0.45),
                min(1, free / 1.3),
                turn / math.pi,
                dist / 5,
                move,
                0.25 * (x - w.ball.x) * direction,
            ]
            if best is None or quality > best[0]:
                best = (quality, j, m.key.id, x, y, data)
    _, j, idx, x, y, data = best
    return replace(base, slot=j, receiver=idx), [x, y], data
