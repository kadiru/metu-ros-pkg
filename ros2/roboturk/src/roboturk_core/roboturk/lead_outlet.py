"""Lead a running outlet and re-evaluate the passing lane at physical release."""

from dataclasses import replace
import math
import numpy as np

from .blocked_outlet import BlockedGame
from .model import RobotKey, body_vector, possession, wrap
from .three_match import CFG
from .three_recipient import move
from .transition_attack import unit


def margin(ball, target, opponents, p):
    """Closest predicted defender center to a rolling ball before reception."""
    dx, dy = unit(target[0] - ball.x, target[1] - ball.y)
    length = math.hypot(target[0] - ball.x, target[1] - ball.y)
    speed = p["roll"]
    best = 2.0
    for rival in opponents:
        rx, ry = rival.x - ball.x, rival.y - ball.y
        vx, vy = speed * dx - p["forecast"] * rival.vx, speed * dy - p["forecast"] * rival.vy
        t = (rx * vx + ry * vy) / max(0.01, vx * vx + vy * vy)
        # A receding challenger behind the ball cannot win the flight by proximity alone.
        if t <= 0:
            continue
        t = min(t, length / speed)
        best = min(best, math.hypot(rx - t * vx, ry - t * vy))
    return best


def rendezvous(w, mate, heading, p):
    b = w.ball
    ux, uy = heading
    travel = math.hypot(mate.x - b.x, mate.y - b.y) / p["roll"]
    lead = float(np.clip(travel * p["run_speed"] * p["horizon"], p["min_lead"], 0.90))
    # The target is the mouth. The robot center stops 0.115 m farther down the ball ray.
    x = float(np.clip(mate.x + lead * ux, -4.08, 4.08))
    y = float(np.clip(mate.y + lead * uy, -2.58, 2.58))
    dx, dy = unit(x - b.x, y - b.y)
    return (x - 0.115 * dx, y - 0.115 * dy), (x, y)


def run_direction(w, mate, p):
    b = w.ball
    direction = w.direction(mate.key.team)
    dx, dy = unit(mate.x - b.x, mate.y - b.y)
    rivals = [r for r in w.robots if r.key.team != mate.key.team]
    best = None
    # Both flanks, a cut toward the ball, and a forward run compete on shared features.
    for hx, hy in [
        (direction, 0),
        (-direction, 0),
        (-dy, dx),
        (dy, -dx),
        (-dx - 0.85 * dy, -dy + 0.85 * dx),
        (-dx + 0.85 * dy, -dy - 0.85 * dx),
        (direction, 1.0),
        (direction, -1.0),
    ]:
        heading = unit(hx, hy)
        target, center = rendezvous(w, mate, heading, p)
        clearance = margin(b, target, rivals, p)
        space = min(math.hypot(center[0] - r.x, center[1] - r.y) for r in rivals)
        score = (
            p["lane_weight"] * min(0.65, clearance)
            + p["progress"] * heading[0] * direction
            + p["space_weight"] * min(1.2, space)
            - 0.5 * (abs(center[1]) > 2.45)
            + p["cut"] * (-dx * heading[0] - dy * heading[1])
        )
        if best is None or score > best[0]:
            best = (score, heading)
    return best[1]


class LeadGame(BlockedGame):
    def __init__(self, *args, **kwargs):
        self.release_plans = {}
        self.run_option = None
        super().__init__(*args, **kwargs)

    def commands(self):
        out = super().commands()
        w, b = self.world, self.world.ball
        if self.flight:
            f = self.flight
            plan = self.release_plans.get(f["tick"])
            if plan:
                mate = w.robot(RobotKey(f["team"], f["receiver"]))
                if not possession(w, mate.key, CFG) and math.hypot(b.vx, b.vy) > 0.24:
                    ux, uy = unit(b.vx, b.vy)
                    x, y = plan["target"]
                    along = max(0.02, (x - b.x) * ux + (y - b.y) * uy)
                    out[mate.key] = move(
                        mate,
                        b.x + (along + 0.115) * ux,
                        b.y + (along + 0.115) * uy,
                        math.atan2(-uy, -ux),
                        speed=1.6,
                    )
            return out
        option = self.option
        if not option or option["action"] != "pass" or self.choice is None:
            return out
        team = option["team"]
        p = self.teams[team]
        if "run_speed" not in p:
            return out
        mate = w.robot(RobotKey(team, option["receiver"]))
        actor = w.robot(RobotKey(team, option["actor"]))
        if self.run_option is not option:
            self.run_option = option
            self.heading = run_direction(w, mate, p)
        target, center = rendezvous(w, mate, self.heading, p)
        dx, dy = unit(target[0] - b.x, target[1] - b.y)
        heading = math.atan2(dy, dx)
        rivals = [r for r in w.robots if r.key.team != team]
        gap = margin(b, target, rivals, p)
        # Make room while the passer turns. Both participants use the same intercept point.
        out[mate.key] = move(mate, *center, math.atan2(-dy, -dx), speed=p["run_speed"])
        original = out[actor.key]
        aim = math.atan2(
            target[1] - b.y + p["aim_time"] * (mate.vy - b.vy),
            target[0] - b.x + p["aim_time"] * (mate.vx - b.vx),
        )
        omega = float(np.clip(5.5 * wrap(aim - actor.yaw), -p["turn"], p["turn"]))
        bx, by = body_vector(b.x - actor.x, b.y - actor.y, actor.yaw)
        gain = omega - original.omega
        running = mate.vx * self.heading[0] + mate.vy * self.heading[1]
        aligned = abs(wrap(heading - actor.yaw)) < p["angle"]
        fire = (
            aligned
            and gap >= p["release_gap"]
            and running >= p["ready"]
            and abs(wrap(heading + math.pi - mate.yaw)) < 0.28
        )
        out[actor.key] = replace(
            original,
            vx=original.vx + gain * by,
            vy=original.vy - gain * bx,
            omega=omega,
            kick=2.2 if fire else 0,
        )
        if fire:
            self.release_plans[self.tick] = dict(
                target=list(target),
                center=list(center),
                runner=mate.key.id,
                heading=list(self.heading),
                clearance=gap,
            )
        return out
