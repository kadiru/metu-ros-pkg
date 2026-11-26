"""Five-player restart behaviors, using field coordinates in metres."""

from dataclasses import replace
import math

from .carrier_motor import MOTOR, track_roller
from .five_decision import pass_speed
from .five_roles import Assignment, CFG, Flight, limit
from .model import Command, possession, unit, wrap
from .three_recipient import move
from .placement_traffic import route


def xy(entity):
    return entity.x, entity.y


def closest(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    t = max(
        0,
        min(
            1,
            ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
            / max(1e-9, dx * dx + dy * dy),
        ),
    )
    return start[0] + t * dx, start[1] + t * dy


def off_ball(r, dest, ball, speed=1.35):
    return replace(move(r, *dest, math.atan2(ball.y - r.y, ball.x - r.x), speed), dribbler=0)


def clear_path(r, start, end, margin=0.75):
    near = closest(xy(r), start, end)
    dx, dy = r.x - near[0], r.y - near[1]
    if math.hypot(dx, dy) >= margin:
        return xy(r)
    if math.hypot(dx, dy) < 0.04:
        dx, dy = -(end[1] - start[1]), end[0] - start[0]
        if math.hypot(dx, dy) < 0.01:
            dx, dy = 1, 0
    dx, dy = unit(dx, dy)
    options = []
    for turn in (0, 0.55, -0.55, 1.05, -1.05, 1.50, -1.50, 2.0, -2.0, math.pi):
        a = math.atan2(dy, dx) + turn
        options.append((near[0] + margin * math.cos(a), near[1] + margin * math.sin(a)))
    for x, y in options:
        if abs(x) < 4.45 and abs(y) < 3.25:
            return x, y
    return limit(options[0][0], 4.4), limit(options[0][1], 3.24)


def stop_formation(w, target=None):
    ball = xy(w.ball)
    end = ball if target is None else target
    return {r.key: off_ball(r, clear_path(r, ball, end, 0.68), w.ball) for r in w.robots}


class KickoffKick:
    """Execute a team kickoff intent and observe when restart restrictions end."""

    def __init__(self, kicker, controller):
        self.kicker = kicker
        self.controller = controller
        self.started_at = None
        self.origin = None
        self.in_play = False
        self.second_touch = False
        self.intent = None
        self.reason = ""

    @property
    def kicked(self):
        return self.origin is not None

    def timed_out(self, now):
        return self.started_at is not None and not self.in_play and now - self.started_at >= 10.0

    def commands(self, world, cfg, tasks):
        if self.started_at is None:
            self.started_at = world.time
        r = world.robot(self.kicker)
        ball = world.ball
        if not self.kicked:
            decision = self.controller.kickoff(world, r, cfg.max_kick)
            self.intent, self.reason = decision.intent, decision.reason
        receiver = self.intent.receiver
        roles, out = {}, {}
        for robot in world.robots:
            key = robot.key
            role, target = "Stand clear", xy(robot)
            task = "Wait until the ball is in play."
            if key == self.kicker:
                role, target = "Kickoff", xy(ball)
                if not self.kicked:
                    task = "Line up the kickoff pass" if receiver else "Wait for an open teammate"
            elif key == receiver:
                role, target = "Kickoff receiver", self.intent.target
                task = "Wait for the kickoff pass, then receive it."
            elif key.team == self.kicker.team:
                role = "Keeper" if key.id == 0 else "Kickoff support"
                task = "Hold your formation position until the ball is in play."
            roles[key] = Assignment(role, target)
            tasks[key] = task
            out[key] = Command()

        planned = {}
        if receiver is not None and not self.kicked:
            target = self.intent.target
            aim = math.atan2(target[1] - ball.y, target[0] - ball.x)
            radial = math.atan2(r.y - ball.y, r.x - ball.x)
            sweep = wrap(aim + math.pi - radial)
            # Orbit the center ball before approaching the mouth contact point.
            theta = radial + limit(sweep, 0.50)
            distance = 0.23 if abs(sweep) > 0.16 else cfg.possession_x
            dest = ball.x + distance * math.cos(theta), ball.y + distance * math.sin(theta)
            out[self.kicker] = replace(move(r, *dest, aim, speed=0.60), dribbler=0)
            if possession(world, self.kicker, cfg) and abs(wrap(aim - r.yaw)) < 0.035:
                speed = min(pass_speed(ball, target, True), cfg.max_kick)
                out[self.kicker] = Command(kick=speed)
                planned[self.kicker] = Flight(self.kicker.team, receiver, target, world.time + 2.15)
        return out, roles, planned

    def observe(self, worlds):
        for world in worlds:
            self.in_play |= math.dist(xy(world.ball), self.origin) >= 0.05
            if self.in_play and any(
                r.key != self.kicker
                and (
                    possession(world, r.key, CFG)
                    or math.dist(xy(world.ball), xy(r)) <= CFG.robot_radius + CFG.ball_radius
                )
                for r in world.robots
            ):
                self.second_touch = True
                break


def withdrawal(w, placer, fallback, margin):
    """Choose a clear outward segment before releasing the dribbler."""
    ball = xy(w.ball)
    start = xy(placer)
    outward = math.atan2(placer.y - w.ball.y, placer.x - w.ball.x)
    best = None
    for step in (0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5, 6, -6):
        angle = outward + step * math.pi / 14
        end = (ball[0] + 0.78 * math.cos(angle), ball[1] + 0.78 * math.sin(angle))
        if abs(end[0]) > 4.78 or abs(end[1]) > 3.32:
            continue
        if (end[0] - start[0]) * (placer.x - w.ball.x) + (end[1] - start[1]) * (
            placer.y - w.ball.y
        ) <= 0:
            continue
        gap = math.inf
        for bot in w.robots:
            if bot.key == placer.key:
                continue
            for t in (0, 0.35):
                obstacle = (bot.x + t * bot.vx, bot.y + t * bot.vy)
                gap = min(gap, math.dist(obstacle, closest(obstacle, start, end)))
        cost = abs(step) * 0.008 + max(0, margin + 0.12 - gap)
        if best is None or cost < best[0]:
            best = cost, end
    return best[1] if best is not None else fallback


class PlacementPlay:
    def __init__(self, world, team, target, weights, traffic=None, clear_retreat=False):
        self.clear_retreat = clear_retreat
        self.traffic = traffic
        self.team, self.target, self.weights = team, target, weights
        squad = [r for r in world.robots if r.key.team == team and r.key.id != 0]
        self.placer = min(squad, key=lambda r: math.dist(xy(r), xy(world.ball))).key
        lateral = -1 if target[1] > 0 else 1
        self.outlet = (
            limit(target[0] + world.direction(team) * 1.25, 3.35),
            limit(target[1] + lateral * 0.92, 2.3),
        )
        self.receiver = min(
            (r for r in squad if r.key != self.placer), key=lambda r: math.dist(xy(r), self.outlet)
        ).key
        self.phase = "capture"
        self.retreat = None
        self.release_yaw = 0.0
        self.kicked = False

    def place(self, w):
        ball = w.ball
        r = w.robot(self.placer)
        dx, dy = self.target[0] - ball.x, self.target[1] - ball.y
        distance = math.hypot(dx, dy)
        chase, cruise, brake, release, settled = self.weights
        angle = math.atan2(ball.y - r.y, ball.x - r.x)
        if self.phase == "retreat" and distance > 0.135:
            self.phase = "capture"
        if self.phase != "retreat":
            if possession(w, r.key, CFG):
                self.phase = "carry"
                if distance < release and math.hypot(ball.vx, ball.vy) < settled:
                    self.phase = "retreat"
                    self.release_yaw = r.yaw
                    self.retreat = clear_path(r, xy(ball), xy(ball), 0.76)
                    if self.clear_retreat:
                        self.retreat = withdrawal(w, r, self.retreat, self.traffic[1])
            else:
                self.phase = "capture"
        out = {}
        roles = {}
        for bot in w.robots:
            dest = clear_path(bot, xy(ball), self.target)
            role = "Clear corridor" if bot.key.team != self.team else "Support"
            if bot.key == self.receiver:
                dest = self.outlet
                role = "Outlet"
            if bot.key == self.placer:
                role = self.phase.capitalize()
                if self.phase == "capture":
                    lead = min(0.25, math.dist(xy(r), xy(ball)) / 1.4)
                    bx, by = ball.x + lead * ball.vx, ball.y + lead * ball.vy
                    dest = (bx - 0.105 * math.cos(angle), by - 0.105 * math.sin(angle))
                    speed = chase if abs(wrap(angle - r.yaw)) < 0.50 else 0.25
                    cmd = move(r, *dest, angle, speed=speed)
                elif self.phase == "carry":
                    ux, uy = unit(dx, dy)
                    speed = min(cruise, brake * distance)
                    cmd = track_roller(w, r, (speed * ux, speed * uy), 0, MOTOR)
                    dest = self.target
                else:
                    dest = self.retreat
                    cmd = replace(move(r, *dest, self.release_yaw, 1.1), dribbler=0)
            else:
                cmd = off_ball(bot, dest, ball)
            roles[bot.key] = Assignment(role, dest)
            out[bot.key] = cmd
        return (route(w, out, self.traffic) if self.traffic else out), roles

    def free_kick(self, w):
        """Get behind the stationary ball, pass, and cushion the reception."""
        ball = w.ball
        passer, receiver = w.robot(self.placer), w.robot(self.receiver)
        ux, uy = unit(receiver.x - ball.x, receiver.y - ball.y)
        aim = math.atan2(uy, ux)
        out = stop_formation(w)
        out[self.receiver] = move(receiver, *self.outlet, wrap(aim + math.pi), 1.1)
        if self.kicked:
            out[self.placer] = Command()
            if possession(w, self.receiver, CFG):
                out[self.receiver] = track_roller(w, receiver, (0, 0), 0, MOTOR)
            else:
                speed = math.hypot(ball.vx, ball.vy)
                if speed > 0.12:
                    fx, fy = unit(ball.vx, ball.vy)
                    ahead = max(
                        0, min(0.7, (receiver.x - ball.x) * fx + (receiver.y - ball.y) * fy)
                    )
                    out[self.receiver] = move(
                        receiver,
                        ball.x + (ahead + 0.109) * fx,
                        ball.y + (ahead + 0.109) * fy,
                        math.atan2(-fy, -fx),
                        1.35,
                    )
            return out
        hx, hy = passer.x - ball.x, passer.y - ball.y
        bearing = math.atan2(hy, hx)
        diff = wrap(aim + math.pi - bearing)
        if abs(diff) > 0.20:
            a = bearing + limit(diff, 0.65)
            dest = (ball.x + 0.25 * math.cos(a), ball.y + 0.25 * math.sin(a))
        else:
            dest = (ball.x - 0.108 * ux, ball.y - 0.108 * uy)
        cmd = replace(move(passer, *dest, aim, 0.85), dribbler=0)
        if possession(w, self.placer, CFG) and abs(wrap(aim - passer.yaw)) < 0.06:
            cmd = replace(cmd, kick=1.75)
        out[self.placer] = cmd
        return out


class PlacementRestart:
    """Track one placement and its subsequent awarded kick across referee heartbeats."""

    def __init__(self, weights, traffic=None, clear_retreat=False, *, referee_places=False):
        self.clear_retreat = clear_retreat
        self.referee_places = referee_places
        self.traffic = traffic
        self.weights = weights
        self.key = None
        self.play = None
        self.direct_since = None
        self.received = 0

    def commands(self, w, state):
        from .model import Referee

        if state.placement_target is not None and state.mode == Referee.STOP:
            key = state.source, state.counter, state.placement_team, state.placement_target
            if self.play is None or self.key != key:
                placement_world = w
                if self.referee_places:
                    x, y = state.placement_target
                    placement_world = replace(w, ball=replace(w.ball, x=x, y=y))
                self.play = PlacementPlay(
                    placement_world,
                    state.placement_team,
                    state.placement_target,
                    self.weights,
                    self.traffic,
                    self.clear_retreat,
                )
                self.key = key
                self.direct_since = None
                self.received = 0
            if self.referee_places:
                return self.stand_clear(w, state.placement_target)
            return self.play.place(w)
        if self.play is None:
            return None
        if state.source != self.key[0] or state.mode == Referee.HALT:
            self.play = None
            return None
        if state.command == "STOP":
            return self.stand_clear(w)
        if state.command != "DIRECT_FREE_" + self.play.team.upper():
            self.play = None
            return None
        if self.direct_since is None:
            self.direct_since = w.time
        self.received = (
            self.received + 1
            if self.play.kicked
            and possession(w, self.play.receiver, CFG)
            and math.hypot(w.ball.vx, w.ball.vy) < 0.35
            else 0
        )
        if self.received >= 5 or w.time - self.direct_since > 12:
            self.play = None
            return None
        out = self.play.free_kick(w)
        if self.traffic:
            out = route(w, out, self.traffic)
        roles = {r.key: Assignment("Stand clear", xy(r)) for r in w.robots}
        roles[self.play.placer] = Assignment("Pass", xy(w.ball))
        roles[self.play.receiver] = Assignment("Receive", self.play.outlet)
        return out, roles

    def stand_clear(self, world, target=None):
        out = stop_formation(world, target)
        roles = {r.key: Assignment("Stand clear", xy(r)) for r in world.robots}
        return (route(world, out, self.traffic) if self.traffic else out), roles

    def applied(self, kicks):
        if self.play and self.direct_since is not None:
            if any(k == self.play.placer for k, _, _ in kicks):
                self.play.kicked = True
