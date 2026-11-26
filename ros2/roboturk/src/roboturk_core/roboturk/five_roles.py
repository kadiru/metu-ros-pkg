"""State-dependent 5v5 tactics. Robot IDs persist as field roles change."""

from dataclasses import dataclass
from itertools import permutations
import math
import random

from . import field as pitch
from .five_strategy import BALANCED
from .model import Ball, Config, Referee, Robot, RobotKey, World, possession
from .spin_release import SpinRelease

CFG = Config(dt=0.04)
TEAMS = ("blue", "yellow")
TRAJECTORY_SAMPLE_DT = 2 * CFG.dt
INTERPOLATION_MARGIN = CFG.max_acceleration * TRAJECTORY_SAMPLE_DT**2 / 4
PENALTY_FRONT = pitch.LENGTH / 2 - pitch.DEFENSE_DEPTH - CFG.robot_radius
PENALTY_HALF_WIDTH = pitch.DEFENSE_WIDTH / 2 + CFG.robot_radius
OFF_BALL_TRACKING_MARGIN = 0.02
ROLE_SWITCH_COST = 0.42
TIMED_ROLES = (
    "Outlet",
    "Marker",
    "Screen",
    "Guard box",
    "Block outlet",
    "Cover",
    "Anchor",
    "Left wing",
    "Right wing",
)
OFF_BALL_ROLES = frozenset((*TIMED_ROLES, "Contain"))


def limit(value, bound):
    return max(-bound, min(bound, value))


def legal_off_ball_target(target):
    """Move a support target to the nearest face outside either padded penalty area."""
    x, y = target
    # Leave room for the timed clearance check and braking overshoot.
    padding = INTERPOLATION_MARGIN + OFF_BALL_TRACKING_MARGIN
    front = PENALTY_FRONT - padding
    half = PENALTY_HALF_WIDTH + padding
    dx = abs(x) - front
    dy = half - abs(y)
    if dx <= 0 or dy <= 0:
        return target
    if dx <= dy:
        return math.copysign(front, x), y
    return x, math.copysign(half, y)


def keeper_target(world, keeper):
    ball = world.ball
    sign = world.direction(keeper.key.team)
    if sign * ball.x < -3.57 and abs(ball.y) < 1.0:
        angle = math.atan2(ball.y - keeper.y, ball.x - keeper.x)
        return ball.x - 0.105 * math.cos(angle), ball.y - 0.105 * math.sin(angle)
    return -4.23 * sign, limit(ball.y * 0.48, 0.49)


def kickoff(seed, serving, direction, time=0.0, *, referee_placement=False):
    rng = random.Random(seed)
    robots = []
    for team in TEAMS:
        sign = direction if team == "blue" else -direction
        kicker_x = -0.115 if team == serving else -0.62
        if referee_placement:
            kicker_x = -0.20 if team == serving else -0.74
        positions = [
            (-4.18, 0),
            (kicker_x, 0),
            (-0.75, 1.55),
            (-1.14, -1.45),
            (-2.45, 0.1),
        ]
        ids = list(range(1, 5))
        rng.shuffle(ids)
        for j, ((x, y), number) in enumerate(zip(positions, [0, *ids], strict=True)):
            if j > 1:
                x += rng.uniform(-0.34, 0.34)
                y += rng.uniform(-0.42, 0.42)
            robots.append(
                Robot(RobotKey(team, number), sign * x, sign * y, 0 if sign > 0 else math.pi)
            )
    return World(
        time, tuple(sorted(robots, key=lambda r: r.key)), Ball(0, 0), Referee.RUNNING, direction
    )


@dataclass(frozen=True)
class Assignment:
    role: str
    target: tuple[float, float]


@dataclass(frozen=True)
class Flight:
    team: str
    receiver: RobotKey
    target: tuple[float, float]
    expires: float
    launch_speed: float | None = None


class FiveRoles:
    def __init__(self):
        self.previous = {}
        self.flight = None
        self.interception = None
        self.control = None
        self.last_touch = -10.0
        self.candidate = None
        self.samples = 0
        self.contested = 0
        self.contain = None
        self.contain_until = 0.0
        self.fetch_stalls = {}
        self.fetcher = None
        self.spin = SpinRelease()

    def observe(self, world, tackling=None):
        holders = [r.key for r in world.robots if possession(world, r.key, CFG)]
        both = len({k.team for k in holders}) == 2
        self.contested = self.contested + 1 if both else 0
        if self.contain and world.time >= self.contain_until:
            self.contain = None
        # Give an enabled tackle a bounded chance to resolve a shared ball.
        # The existing containment rule still ends a prolonged double grip.
        tackle_duel = tackling in holders and self.control in holders and tackling != self.control
        hold_ticks = 20 if tackle_duel else 4
        if both and tackle_duel and self.contested < hold_ticks:
            self.last_touch = world.time
        if self.contested >= hold_ticks and self.contain is None:
            # End a failed challenge by having the challenger yield the lane.
            # Without an assigned challenger, the weaker grip yields.
            def grip_error(key):
                r = world.robot(key)
                return math.hypot(
                    world.ball.x - r.x - CFG.possession_x * math.cos(r.yaw),
                    world.ball.y - r.y - CFG.possession_x * math.sin(r.yaw),
                )

            field_players = [k for k in holders if k.id != 0]
            if field_players:
                self.contain = (
                    tackling
                    if tackle_duel and tackling in field_players
                    else max(field_players, key=grip_error)
                )
                self.contain_until = world.time + 0.72
                self.contested = 0
        sole = holders[0] if len(holders) == 1 else None
        self.spin.observe(world, holders, sole)
        self.samples = self.samples + 1 if sole == self.candidate else 1
        self.candidate = sole
        if sole and self.samples >= 3:
            if sole == self.contain:
                self.contain = None
            self.control = sole
            self.last_touch = world.time
            if self.flight and (sole.team != self.flight.team or sole == self.flight.receiver):
                self.flight = None
        if self.flight and world.time >= self.flight.expires:
            self.flight = None
        if world.time - self.last_touch > 0.32:
            self.control = None
        return sole

    def assign(self, world, team, legal_off_ball=True, audit=None, strategy=BALANCED, controller=None):
        if controller is None:
            from .five_decision import CONTROLLERS

            controller = CONTROLLERS[strategy.controller]
        players = sorted((r for r in world.robots if r.key.team == team), key=lambda r: r.key.id)
        if len(players) != 5 or players[0].key.id != 0:
            raise ValueError("Five-player teams require a designated goalkeeper with ID 0")
        sign = world.direction(team)
        bx, by = sign * world.ball.x, sign * world.ball.y
        wing_width = 1.45 + strategy.book.wing_width
        attacking_wing = 1.32 + strategy.book.wing_advance

        def field(x, y):
            return sign * limit(x, 4.30), sign * limit(y, 2.73)

        keeper = players[0].key
        positions = {keeper: Assignment("Keeper", keeper_target(world, players[0]))}
        team_in_control = (
            self.flight.team if self.flight else self.control.team if self.control else None
        )
        forced = None
        spin_outlet = None
        if team_in_control == team:
            phase = "Attack"
            if self.flight:
                primary = "Receiver"
                main = self.flight.target
                forced = self.flight.receiver
            elif self.control != keeper:
                primary = "Carrier"
                main = (world.ball.x, world.ball.y)
                forced = self.control
            else:
                primary = "Outlet"
                main = field(bx + 1.45, by * 0.3)
            left, right = controller.wings(
                world, team,
                field(bx + attacking_wing, max(0.80, wing_width + 0.34 * by)),
                field(bx + attacking_wing, min(-0.80, -wing_width + 0.34 * by)),
            )
            roles = {
                primary: main,
                "Left wing": left,
                "Right wing": right,
                "Anchor": field(bx - 1.35, by * 0.48),
            }
            turn = self.spin.turn
            if turn and turn.actor == forced and turn.receiver:
                spin_outlet = turn.receiver
                del roles["Anchor"]
                roles["Spin outlet"] = turn.target
        elif team_in_control is not None:
            phase = "Defend"
            rivals = [
                r
                for r in world.robots
                if r.key.team != team and r.key.id != 0 and r.key != self.control
            ]
            threat = min(rivals, key=lambda r: math.hypot(sign * r.x + 4.5, r.y))
            roles = {
                "Presser": field(bx, by),
                "Marker": field(sign * threat.x - 0.31, sign * threat.y * 0.94),
                "Screen": field(bx * 0.64 - 1.40, by * 0.64),
                "Cover": field(bx * 0.28 - 2.94, by * 0.28),
            }
        else:
            phase = "Recover"
            roles = {
                "Chaser": field(bx, by),
                "Left wing": field(bx + 0.65, wing_width + 0.25 * by),
                "Right wing": field(bx + 0.65, -wing_width + 0.25 * by),
                "Cover": field(bx * 0.45 - 2.08, by * 0.42),
            }
        if phase != "Attack" and abs(bx) > 3.64 and abs(by) < 0.99:
            # Leave room for the designated keeper to collect. The nearest field
            # player covers the return lane at the edge of the goal area.
            primary = next(iter(roles))
            roles = {
                "Block outlet" if bx > 0 else "Guard box": field(math.copysign(3.48, bx), by),
                **{n: p for n, p in roles.items() if n != primary},
            }
        if self.interception and self.interception.actor.team == team and phase == "Defend":
            primary = next(iter(roles))
            roles = {
                "Interceptor": self.interception.center,
                **{n: p for n, p in roles.items() if n != primary},
            }
            forced = self.interception.actor
        elif self.contain and self.contain.team == team and phase != "Attack":
            primary = next(iter(roles))
            roles = {
                "Contain": field(bx - 0.54, by * 0.94),
                **{n: p for n, p in roles.items() if n != primary},
            }
            forced = self.contain
        if legal_off_ball:
            roles = {
                name: legal_off_ball_target(target) if name in OFF_BALL_ROLES else target
                for name, target in roles.items()
            }
        names = tuple(roles)
        # The small roster permits an exact minimum-cost bijection, with a travel
        # penalty for giving up a role. Possession and intended reception take priority.
        costs = {
            (r.key, name): math.dist((r.x, r.y), target)
            + (ROLE_SWITCH_COST if r.key in self.previous and self.previous[r.key] != name else 0)
            for r in players[1:]
            for name, target in roles.items()
        }
        allowed = (
            order
            for order in permutations(players[1:])
            if (forced is None or order[0].key == forced)
            and (spin_outlet is None or order[-1].key == spin_outlet)
        )
        match = controller.lineup(allowed, names, costs)
        positions.update({r.key: Assignment(n, roles[n]) for r, n in zip(match, names)})
        if audit is not None:
            roster_cost = sum(costs[r.key, name] for r, name in zip(match, names, strict=True))
            audit[keeper] = dict(basis="keeper", allocator_previous=self.previous.get(keeper))
            for r, name in zip(match, names, strict=True):
                previous = self.previous.get(r.key)
                audit[r.key] = dict(
                    basis="spin_outlet"
                    if r.key == spin_outlet
                    else name.lower()
                    if r.key == forced
                    else controller.assignment_basis,
                    allocator_previous=previous,
                    distance_m=round(math.dist((r.x, r.y), roles[name]), 4),
                    switch_cost=ROLE_SWITCH_COST if previous and previous != name else 0,
                    roster_cost=round(roster_cost, 4),
                )
        self.previous.update({k: a.role for k, a in positions.items()})
        return phase, positions
