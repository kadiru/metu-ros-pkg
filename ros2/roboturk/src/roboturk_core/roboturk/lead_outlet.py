"""Lead a running outlet and re-evaluate the passing lane at physical release."""

from dataclasses import replace
import math
import numpy as np

from .blocked_outlet import BlockedGame, geometry
from .model import RobotKey, body_vector, possession, wrap
from .three_match import CFG
from .three_recipient import candidates, move
from .transition_attack import unit, trap


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
    reach = p.get("reach", 0)
    if reach:
        # After the kick defenders can abandon their current heading and chase the pass.
        # Check the entire flight and the receiver's first touch at the calibrated roll speed.
        for j in range(1, 9):
            t = (j / 8) * length / speed
            x, y = ball.x + dx * t * speed, ball.y + dy * t * speed
            for rival in opponents:
                lag = min(t, 0.10)
                gap = math.hypot(x - rival.x - lag * rival.vx, y - rival.y - lag * rival.vy)
                best = min(best, gap - reach * max(0, t - 0.10))
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
            - p.get("length_weight", 0) * abs(math.hypot(target[0] - b.x, target[1] - b.y) - 1.0)
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
                p = self.teams[mate.key.team]
                if possession(w, mate.key, CFG) and p.get("receive_escape", 0):
                    rival = min(
                        (r for r in w.robots if r.key.team != mate.key.team),
                        key=lambda r: math.hypot(r.x - b.x, r.y - b.y),
                    )
                    ux, uy = unit(b.x - rival.x, b.y - rival.y)
                    v = p["receive_escape"] * max(
                        0, 1 - math.hypot(rival.x - b.x, rival.y - b.y) / 0.9
                    )
                    out[mate.key] = trap(w, mate, mate.yaw, (v * ux, v * uy), p)
                elif not possession(w, mate.key, CFG) and math.hypot(b.vx, b.vy) > 0.24:
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
            if p.get("choose_route"):
                rivals = [r for r in w.robots if r.key.team != team]
                best = None
                scores = []
                for slot, runner in enumerate(candidates(w, actor)):
                    h = run_direction(w, runner, p)
                    point, _ = rendezvous(w, runner, h, p)
                    _, lane, space = geometry(
                        (b.x, b.y),
                        w.direction(team),
                        (runner.x, runner.y),
                        [(r.x, r.y) for r in rivals],
                    )
                    value = (
                        2.8 * min(0.75, margin(b, point, rivals, p))
                        + 0.65 * min(1.3, space)
                        - 3.5 * max(0, 0.30 - lane)
                        - 2 * max(0, 0.60 - space)
                        - 0.22 * abs(math.hypot(point[0] - b.x, point[1] - b.y) - 1.1)
                        + 0.17 * (runner.key == mate.key)
                    )
                    scores.append(float(value))
                    if best is None or value > best[0]:
                        best = (value, runner, h, slot)
                _, mate, self.heading, slot = best
                probability = float(
                    1 / (1 + math.exp(float(np.clip(scores[0] - scores[1], -30, 30))))
                )
                self.target_choice = replace(
                    self.choice, receiver=mate.key.id, slot=slot, probability=probability
                )
                option["route_scores"] = scores
                self.choice = self.target_choice
                option["receiver"] = mate.key.id
        if (
            p.get("replan")
            and self.tick - option["tick"] > 0
            and (self.tick - option["tick"]) % 8 == 0
        ):
            self.heading = run_direction(w, mate, p)
        target, center = rendezvous(w, mate, self.heading, p)
        dx, dy = unit(target[0] - b.x, target[1] - b.y)
        heading = math.atan2(dy, dx)
        rivals = [r for r in w.robots if r.key.team != team]
        gap = margin(b, target, rivals, p)
        # Make room while the passer turns. Both participants use the same intercept point.
        prepared = abs(wrap(heading - actor.yaw)) < p.get("sync_angle", math.pi + 0.01)
        if not prepared or (
            p.get("stand_off", 0) and math.hypot(target[0] - b.x, target[1] - b.y) < p["stand_off"]
        ):
            center = (mate.x, mate.y)
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
