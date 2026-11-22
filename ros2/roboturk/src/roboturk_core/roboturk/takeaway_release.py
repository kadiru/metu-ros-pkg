"""Finish a hurried recovery or clear the passing lane for an immediate outlet."""

from dataclasses import replace
import math
from .carrier_motor import track_roller
from .model import wrap
from .three_match import face
from .three_recipient import candidates, choose, distance, move
from .transition_attack import unit


def shot(w, actor, profile):
    b = w.ball
    gx = 4.53 * w.direction(actor.key.team)
    targets = []
    for y in [0, -0.18, 0.18, -0.32, 0.32, -0.42, 0.42]:
        angle = math.atan2(y - b.y, gx - b.x)
        dx, dy = math.cos(angle), math.sin(angle)
        clearance = 1.5
        for q in w.robots:
            if q.key == actor.key:
                continue
            along = (q.x - b.x) * dx + (q.y - b.y) * dy
            if 0.02 < along < math.hypot(y - b.y, gx - b.x) + 0.1:
                dt = min(0.68, along / 3.35)
                clearance = min(
                    clearance, abs((q.x + dt * q.vx - b.x) * dy - (q.y + dt * q.vy - b.y) * dx)
                )
        targets.append((min(0.36, clearance) - 0.035 * abs(y), angle))
    _, angle = max(targets, key=lambda t: t[0])
    enemy = min((q for q in w.robots if q.key.team != actor.key.team), key=lambda q: distance(b, q))
    ex, ey = unit(b.x - enemy.x, b.y - enemy.y)
    ux, uy = unit(gx - b.x, -b.y)
    speed = profile["escape"] * max(0, 1 - distance(enemy, b) / 0.80)
    direction = w.direction(actor.key.team)
    crossing = b.y + (gx - b.x) * math.tan(actor.yaw)
    # Release on the first control tick that a moving roller can put the
    # ball inside the goal, using the yaw observed at that actual tick.
    open_goal = math.cos(actor.yaw) * direction > 0.1 and abs(crossing) < profile["margin"]
    cmd = track_roller(
        w,
        actor,
        (0.54 * ux + speed * ex, 0.54 * uy + speed * ey),
        24.0 * wrap(angle - actor.yaw),
        dict(drive=0.64, step_v=0.48, grip=25, pivot=8, cap=1.88),
    )
    return replace(cmd, kick=4.0 if open_goal else 0.0)


def release(game, out, actor, profile):
    w = game.world
    b = w.ball
    cmd = shot(w, actor, profile)
    game.owner = actor.key
    game.choice = None
    game.release_plans.pop(game.tick, None)
    out[actor.key] = cmd
    if not profile["outlet"] or cmd.kick or actor.key.id != game.counter["actor"]:
        return out
    mates = candidates(w, actor)
    options = []
    for i, mate in enumerate(mates):
        length = distance(mate, b)
        if length < 0.68:
            continue
        angle = math.atan2(mate.y - b.y, mate.x - b.x)
        err = abs(wrap(angle - actor.yaw))
        if err < 0.38:
            options.append((err, i, angle, length))
    if not options:
        return out
    _, i, angle, length = min(options)
    mate = mates[i]
    dx, dy = math.cos(actor.yaw), math.sin(actor.yaw)
    point = (b.x + (length + 0.115) * dx, b.y + (length + 0.115) * dy)
    out[mate.key] = move(mate, *point, face(mate, b), speed=1.24)
    other = mates[1 - i]
    if abs((other.x - b.x) * dy - (other.y - b.y) * dx) < 0.22:
        sign = 1 if (other.x - b.x) * (-dy) + (other.y - b.y) * dx >= 0 else -1
        out[other.key] = move(
            other,
            other.x - 0.58 * sign * dy,
            other.y + 0.58 * sign * dx,
            face(other, b),
            speed=1.45,
        )
    cmd = track_roller(
        w,
        actor,
        (b.vx, b.vy),
        24 * wrap(angle - actor.yaw),
        dict(drive=0.65, step_v=0.45, grip=24, pivot=8, cap=1.88),
    )
    game.choice = replace(
        choose(w, actor, game.weights[actor.key.team]), slot=i, receiver=mate.key.id
    )
    out[actor.key] = replace(
        cmd, kick=2.2 if abs(wrap(angle - actor.yaw)) * length < profile["accuracy"] else 0
    )
    return out
