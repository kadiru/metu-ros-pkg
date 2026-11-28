"""Recycle a crowded ball through a rear outlet and offer two wide returns."""

from dataclasses import dataclass
import math

from .carrier_motor import MOTOR, track_roller
from .counterpass import clearance
from .five_decision import AttackDecision, pass_options, pass_speed, pass_target
from .five_roles import Assignment, Flight, legal_off_ball_target, limit
from .model import Command, Intent, RobotKey, Skill, body_vector, possession, unit, wrap
from .spin_release import arc_clear


def blockers(world, passer, receiver):
    return [
        robot for robot in world.robots if robot.visible and robot.key not in (passer, receiver)
    ]


def lane(world, passer, receiver, target, speed):
    return clearance(world.ball.x, world.ball.y, *target, blockers(world, passer, receiver), speed)


@dataclass
class Relay:
    passer: RobotKey
    outlet: RobotKey
    partner: RobotKey
    target: tuple[float, float]
    started: float
    phase: str = "turn"
    received_at: float | None = None
    wing_targets: dict | None = None

    @classmethod
    def choose(cls, world, actor, weights):
        team = actor.key.team
        sign = world.direction(team)
        ball = world.ball
        rivals = [r for r in world.robots if r.key.team != team and r.visible]
        if (
            actor.key.id == 0
            or min((math.dist((r.x, r.y), (ball.x, ball.y)) for r in rivals), default=math.inf)
            > 0.70
        ):
            return None
        forward = list(pass_options(world, actor, weights))
        if not forward or any(
            lane(world, actor.key, mate.key, pass_target(mate), 2.3) >= 0.20
            for _, mate, _ in forward
        ):
            return None
        _, partner, _ = max(forward, key=lambda option: option[0])
        choices = []
        for mate in world.robots:
            if (
                not mate.visible
                or mate.key.team != team
                or not mate.key.id
                or mate.key == actor.key
            ):
                continue
            distance = math.dist((ball.x, ball.y), (mate.x, mate.y))
            progress = sign * (mate.x - actor.x)
            if not (-2.1 < progress < -0.60 and 0.80 < distance < 2.6):
                continue
            target = pass_target(mate)
            gap = lane(world, actor.key, mate.key, target, pass_speed(ball, target, True))
            room = min(math.dist(target, (r.x, r.y)) for r in rivals)
            if gap > 0.23 and room > 0.85:
                choices.append((min(gap, 0.60) + 0.35 * min(room, 1.6), mate.key, target))
        if not choices:
            return None
        _, outlet, target = max(choices)
        return cls(actor.key, outlet, partner.key, target, world.time)

    def advance(self, world, control, sole, flight):
        if self.phase == "turn":
            if flight and flight.receiver == self.outlet:
                self.phase = "receive"
                passer, partner = world.robot(self.passer), world.robot(self.partner)
                sign = world.direction(self.passer.team)
                x = limit(world.ball.x + sign * 1.8, 3.6)
                high, low = sorted((passer, partner), key=lambda robot: robot.y, reverse=True)
                self.wing_targets = {high.key: (x, 2.22), low.key: (x, -2.22)}
                return True
            passer = world.robot(self.passer)
            ball = world.ball
            forward, sideways = body_vector(ball.x - passer.x, ball.y - passer.y, passer.yaw)
            recovering_grip = sole is None and abs(forward - 0.115) < 0.065 and abs(sideways) < 0.06
            return (
                world.time - self.started < 1.8
                and control == self.passer
                and (sole == self.passer or recovering_grip)
            )
        if self.phase == "receive":
            if control == sole == self.outlet and flight is None:
                self.phase = "return"
                self.received_at = world.time
                return True
            return (
                flight is not None
                and flight.receiver == self.outlet
                and world.time - self.started < 3.5
            )
        return (
            control == sole == self.outlet
            and flight is None
            and world.time - self.received_at < 2.2
        )

    def assignments(self, world, assignments):
        if self.phase == "turn":
            assignments[self.outlet] = Assignment("Backpass outlet", self.target)
        else:
            sign = world.direction(self.passer.team)
            for runner, target in self.wing_targets.items():
                role = "Left wing" if target[1] * sign > 0 else "Right wing"
                assignments[runner] = Assignment(role, legal_off_ball_target(target))
            other = next(
                key
                for key in assignments
                if key.team == self.passer.team
                and key.id
                and key not in (self.outlet, self.passer, self.partner)
            )
            assignments[other] = Assignment(
                "Anchor",
                legal_off_ball_target((limit(world.ball.x - sign * 0.95, 4.3), world.ball.y * 0.4)),
            )

    def backpass(self, world, actor, cfg, measured, receive):
        ball = world.ball
        aim = math.atan2(self.target[1] - ball.y, self.target[0] - ball.x)
        error = wrap(aim - actor.yaw)
        kick = min(cfg.max_kick, pass_speed(ball, self.target, receive))
        if (
            possession(world, actor.key, cfg)
            and abs(error) < 0.068
            and abs(actor.omega) < 0.55
            and lane(world, actor.key, self.outlet, self.target, kick) > 0.20
        ):
            return (
                Command(dribbler=1, kick=kick),
                Flight(actor.key.team, self.outlet, self.target, world.time + 2.15),
                "Pass back to the open outlet",
            )
        nearest = min(
            (r for r in world.robots if r.key.team != actor.key.team),
            key=lambda r: math.dist((ball.x, ball.y), (r.x, r.y)),
        )
        away = unit(ball.x - nearest.x, ball.y - nearest.y)
        distance = math.dist((ball.x, ball.y), (nearest.x, nearest.y))
        forward, sideways = body_vector(ball.x - actor.x, ball.y - actor.y, actor.yaw)
        grip = max(abs(forward - cfg.possession_x) / 0.035, abs(sideways) / 0.045)
        rate = min(4.2, cfg.max_omega) * max(0.25, min(1.0, (1.10 - grip) / 0.45))
        sweep = limit(error, rate * (2 * cfg.dt))
        turning = arc_clear(
            world,
            actor,
            sweep,
            rate,
            cfg,
            measured,
            2 * cfg.robot_radius + 0.028,
            shield_escape=True,
        )
        desired_speed = (0.76 if distance < 0.48 else 0.16) * max(0.25, 1.0 - 0.75 * grip)
        desired = tuple(value * desired_speed for value in away)
        requested = limit(7.2 * error, rate)
        omega = actor.omega + limit(requested - actor.omega, 18 * cfg.dt) if turning else 0.0
        return (
            track_roller(world, actor, desired, omega, MOTOR),
            None,
            (
                "Turn with the ball toward the rear outlet"
                if turning
                else "Shield and settle for the backpass"
            ),
        )

    def return_pass(self, world, actor, receive):
        for runner in sorted(
            self.wing_targets,
            key=lambda k: math.dist(self.wing_targets[k], (world.robot(k).x, world.robot(k).y)),
        ):
            target = pass_target(world.robot(runner))
            if abs(target[1]) < 1.25 or math.dist(target, self.wing_targets[runner]) > 0.85:
                continue
            if (
                lane(world, actor.key, runner, target, pass_speed(world.ball, target, receive))
                > 0.27
            ):
                return AttackDecision(
                    Intent(Skill.PASS, target, runner), "Return the ball to the open wing."
                )
        return None
