"""Observable context at the first exclusive touch of a midfield contest."""

import math
from .model import body_vector
from .three_recipient import distance
from .transition_attack import unit


def first_touch_features(w, actor):
    b = w.ball
    opponent = min(
        (r for r in w.robots if r.key.team != actor.key.team), key=lambda r: distance(r, b)
    )
    nx, ny = unit(opponent.x - actor.x, opponent.y - actor.y)
    tx, ty = -ny, nx
    bx, by = b.x - actor.x, b.y - actor.y
    sign = math.copysign(1, bx * tx + by * ty or 1)
    tx *= sign
    ty *= sign
    forward, lateral = body_vector(bx, by, actor.yaw)
    ox, oy = body_vector(b.x - opponent.x, b.y - opponent.y, opponent.yaw)
    dx, dy = b.vx - actor.vx, b.vy - actor.vy
    mates = [r for r in w.robots if r.key.team == actor.key.team and r.key != actor.key]
    return [
        math.hypot(b.vx, b.vy),
        b.vx * nx + b.vy * ny,
        b.vx * tx + b.vy * ty,
        forward,
        abs(lateral),
        bx * tx + by * ty,
        ox,
        abs(oy),
        distance(actor, opponent),
        dx * nx + dy * ny,
        dx * tx + dy * ty,
        actor.omega * sign,
        opponent.omega * sign,
        math.cos(actor.yaw) * nx + math.sin(actor.yaw) * ny,
        math.cos(opponent.yaw) * tx + math.sin(opponent.yaw) * ty,
        min(distance(b, r) for r in mates),
        max(distance(b, r) for r in mates),
    ]


def select_motor(tree, x):
    while "action" not in tree:
        tree = tree["left"] if x[tree["feature"]] <= tree["threshold"] else tree["right"]
    return tree["action"]
