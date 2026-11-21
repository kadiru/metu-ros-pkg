"""First-touch control and team decisions for a 3v3 attack after a contested ball."""

import math
import numpy as np
from .jaw_recovery import JawGame
from .model import Command, RobotKey, body_vector, possession, wrap
from .recovery_attack import AttackGame, PossessionEnded
from .recovery_lead import route
from .three_match import CFG, face
from .three_recipient import distance, move

ACTIONS = ("pass", "dribble", "shoot")


def unit(dx, dy):
    length = max(1e-6, math.hypot(dx, dy))
    return dx / length, dy / length


def window(w, actor):
    """Evaluate the space a 4 m/s ball has at each player's predicted crossing."""
    b = w.ball
    gx = 4.53 * w.direction(actor.key.team)
    best = None
    for gy in (0.0, -0.16, 0.16, -0.31, 0.31):
        ux, uy = unit(gx - b.x, gy - b.y)
        length = math.hypot(gx - b.x, gy - b.y)
        gap = 1.0
        for r in w.robots:
            if r.key == actor.key:
                continue
            rx, ry = r.x - b.x, r.y - b.y
            along = rx * ux + ry * uy
            if not -0.02 < along < length + 0.12:
                continue
            t = min(0.55, max(0.0, along) / 3.7)
            offset = rx * uy - ry * ux
            future = offset + t * (r.vx * uy - r.vy * ux)
            gap = min(gap, abs(future), abs(offset) + 0.08)
        quality = min(0.44, gap) - 0.12 * abs(gy)
        if best is None or quality > best[0]:
            best = (quality, gy, gap)
    return gx, best[1], best[2]


def trap(w, r, angle, velocity, params, kick=0.0):
    """Control mouth velocity around the moving ball while rotating the chassis."""
    bx, by = body_vector(w.ball.x - r.x, w.ball.y - r.y, r.yaw)
    bvx, bvy = body_vector(w.ball.vx, w.ball.vy, r.yaw)
    ux, uy = body_vector(*velocity, r.yaw)
    alpha = params["tracking"]
    kp = params["centering"]
    omega = float(np.clip(5.5 * wrap(angle - r.yaw), -params["turn"], params["turn"]))
    return Command(
        float(np.clip((1 - alpha) * bvx + alpha * ux + kp * (bx - 0.115) + omega * by, -1.8, 1.8)),
        float(np.clip((1 - alpha) * bvy + alpha * uy + kp * by - omega * bx, -1.8, 1.8)),
        omega,
        1,
        kick,
    )


def separation(w, actor, rival, scale):
    dx, dy = unit(w.ball.x - rival.x, w.ball.y - rival.y)
    speed = scale * max(0.0, 1.0 - distance(w.ball, rival) / 0.7)
    return dx * speed, dy * speed


def carry_velocity(w, actor, gx, gy, rival, params):
    """Move the ball toward goal while avoiding projected bodies in the next metre."""
    b = w.ball
    heading = math.atan2(gy - b.y, gx - b.x)
    speed = params["speed"]
    best = None
    for a in (0.0, -0.38, 0.38, -0.78, 0.78, -1.22, 1.22):
        dx, dy = math.cos(heading + a), math.sin(heading + a)
        gap = 0.70
        for r in w.robots:
            if r.key == actor.key:
                continue
            rx, ry = r.x - b.x, r.y - b.y
            along = rx * dx + ry * dy
            if -0.015 < along < 1.15:
                gap = min(gap, abs(rx * dy - ry * dx))
        boundary = (
            abs(b.y + dy * 0.6) > 2.63 or (b.x + dx * 0.6) * w.direction(actor.key.team) > 4.12
        )
        quality = 1.6 * min(0.55, gap) + 0.46 * math.cos(a) - float(boundary)
        if best is None or quality > best[0]:
            best = (quality, dx, dy)
    ex, ey = separation(w, actor, rival, params["escape"])
    return speed * best[1] + ex, speed * best[2] + ey


class TransitionGame(AttackGame):
    def __init__(self, *args, transition, **kwargs):
        self.transition = transition
        super().__init__(*args, **kwargs)

    def commands(self):
        if self.early_stop and self.terminal_restart:
            raise PossessionEnded
        out = JawGame.commands(self)
        previous = self.active
        self.observe_recovery()
        if self.early_stop and previous and not self.active:
            raise PossessionEnded
        w = self.world
        held = [r for r in w.robots if possession(w, r.key, CFG)]
        if len(held) != 1 or self.flight is not None:
            self.option = None
            return out
        actor = held[0]
        team = actor.key.team
        p = self.transition
        rival = min((r for r in w.robots if r.key.team != team), key=lambda r: distance(r, w.ball))
        gx, gy, gap = window(w, actor)
        shot = math.atan2(gy - w.ball.y, gx - w.ball.x)
        escape = separation(w, actor, rival, p["escape"])
        if self.active is None or self.active[0] != team:
            if actor.key in self.contestants:
                out[actor.key] = trap(w, actor, shot, escape, p)
            return out
        _, recovery = self.active
        if (
            not self.option
            or self.option["actor"] != actor.key.id
            or self.tick >= self.option["until"]
        ):
            choice, target, lane = route(w, actor, self.weights[team])
            remaining = abs(gx - w.ball.x)
            pressure = distance(w.ball, rival)
            alignment = math.cos(shot - actor.yaw)
            # Opening and continuation use the same policy. The receiver re-evaluates the field.
            features = [1.0, gap, min(7.0, remaining) / 7, alignment, min(1.0, pressure), *lane]
            logits = [
                p["pass_bias"] + 1.15 * lane[0] + 0.48 * lane[-1] - 0.55 * lane[2] - 1.3 * gap,
                p["carry_bias"] + 0.6 * remaining / 7 - 0.9 * gap,
                p["shot_bias"] + 2.6 * min(0.75, gap) + 0.22 * alignment - 0.58 * remaining / 7,
            ]
            if "weights" in p:
                logits = (np.asarray(p["weights"]) @ features).tolist()
            action = ACTIONS[int(np.argmax(logits))]
            attempts = [o for o in self.options if o["recovery"] == recovery and o["team"] == team]
            if self.opening_action is not None and not attempts:
                action = ACTIONS[self.opening_action]
            self.option = dict(
                team=team,
                recovery=recovery,
                tick=self.tick,
                actor=actor.key.id,
                action=action,
                x=features,
                values=logits,
                receiver=choice.receiver,
                target=target,
                until=self.tick + (15 if action == "dribble" else 60),
            )
            self.options.append(self.option)
            self.target_choice = choice
        action = self.option["action"]
        self.owner = actor.key
        self.choice = None
        if action == "pass":
            self.choice = self.target_choice
            mate = w.robot(RobotKey(team, self.choice.receiver))
            x, y = self.option["target"]
            out[mate.key] = move(mate, x, y, face(mate, w.ball))
            angle = math.atan2(y - w.ball.y, x - w.ball.x)
            out[actor.key] = trap(
                w, actor, angle, escape, p, 2.2 if abs(wrap(angle - actor.yaw)) < 0.028 else 0.0
            )
        elif action == "dribble":
            v = carry_velocity(w, actor, gx, gy, rival, p)
            out[actor.key] = trap(w, actor, shot, v, p)
        else:
            # A fast release is possible anywhere within the free central goal aperture.
            angle = actor.yaw
            same_direction = math.cos(angle) * w.direction(team) > 0.1
            projected = w.ball.y + (gx - w.ball.x) * math.tan(angle) if same_direction else 10.0
            aligned = abs(wrap(shot - angle)) < 0.017 or (abs(projected - gy) < 0.12 and gap > 0.24)
            out[actor.key] = trap(w, actor, shot, escape, p, 4.0 if aligned else 0.0)
            ux, uy = unit(gx - w.ball.x, gy - w.ball.y)
            for mate in w.robots:
                if mate.key.team != team or mate.key == actor.key:
                    continue
                rx, ry = mate.x - w.ball.x, mate.y - w.ball.y
                along = rx * ux + ry * uy
                cross = ry * ux - rx * uy
                if 0 < along < remaining_distance(w, gx, gy) and abs(cross) < 0.65:
                    side = 1 if cross >= 0 else -1
                    out[mate.key] = move(
                        mate,
                        mate.x - side * 0.72 * uy,
                        mate.y + side * 0.72 * ux,
                        face(mate, w.ball),
                    )
        return out


def remaining_distance(w, x, y):
    return math.hypot(x - w.ball.x, y - w.ball.y)
