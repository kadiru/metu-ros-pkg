"""Team-local 5v5 decisions. The match owns skill execution and safety."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from itertools import product
import math
from typing import TYPE_CHECKING, Protocol

from .counterpass import clearance, shot_window
from .model import Intent, Robot, RobotKey, Skill, World
from .three_recipient import features

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from .five_strategy import TeamStrategy


Point = tuple[float, float]
Lineup = tuple[Robot, ...]
RoleCosts = Mapping[tuple[RobotKey, str], float]


@dataclass(frozen=True)
class AttackDecision:
    intent: Intent
    reason: str = ""


class TeamController(Protocol):
    assignment_basis: str

    def lineup(
        self, eligible: Iterable[Lineup], roles: tuple[str, ...], costs: RoleCosts
    ) -> Lineup:
        """Choose from lineups that preserve reserved carrier, receiver and contain roles."""
        ...

    def wings(self, world: World, team: str, left: Point, right: Point) -> tuple[Point, Point]:
        """Choose support targets during attack. The match applies field constraints."""
        ...

    def attack(
        self, world: World, carrier: Robot, strategy: TeamStrategy, weights: NDArray, keeper: bool
    ) -> AttackDecision:
        """Propose a pass, dribble, or shot for the shared carrier skills."""
        ...


def pass_target(mate: Robot) -> Point:
    return (
        max(-4.22, min(4.22, mate.x + 0.16 * mate.vx)),
        max(-2.68, min(2.68, mate.y + 0.16 * mate.vy)),
    )


def pass_options(world, carrier, weights):
    team = carrier.key.team
    sign = world.direction(team)
    for mate in world.robots:
        if mate.key.team != team or mate.key == carrier.key or mate.key.id == 0:
            continue
        progress = sign * (mate.x - carrier.x)
        if progress >= -0.60 and 0.72 < math.hypot(mate.x - carrier.x, mate.y - carrier.y) < 4.1:
            yield float(features(world, carrier, mate) @ weights) + 0.8 * progress, mate, progress


class BalancedController:
    assignment_basis = "roster_cost"

    def lineup(self, eligible, roles, costs):
        return min(eligible, key=lambda order: sum(costs[r.key, n] for r, n in zip(order, roles)))

    def wings(self, world, team, left, right):
        return left, right

    def attack(self, world, carrier, strategy, weights, keeper):
        team = carrier.key.team
        ball, sign = world.ball, world.direction(team)
        others = [r for r in world.robots if r.key != carrier.key]
        gx, gy, gap = shot_window(ball.x, ball.y, sign, others)
        if sign * (gx - ball.x) < strategy.book.shoot_range and gap > 0.13:
            return AttackDecision(Intent(Skill.SHOOT, (gx, gy)))
        pressure = min(math.hypot(r.x - ball.x, r.y - ball.y) for r in others if r.key.team != team)
        choice = max(pass_options(world, carrier, weights), key=lambda row: row[0], default=None)
        if choice and strategy.pass_is_available(keeper, pressure, choice[2]):
            mate = choice[1]
            return AttackDecision(Intent(Skill.PASS, pass_target(mate), mate.key))
        return AttackDecision(Intent(Skill.DRIBBLE, (gx, gy)))


class LocalPriorityController(BalancedController):
    assignment_basis = "role_priority"

    def lineup(self, eligible, roles, costs):
        # A lexicographic choice gives each role first pick from robots that can
        # fill it while preserving the reservations in the remaining slots.
        return min(eligible, key=lambda order: tuple(costs[r.key, n] for r, n in zip(order, roles)))


class CoordinatedController(BalancedController):
    def wings(self, world, team, left, right):
        ball = world.ball
        rivals = [r for r in world.robots if r.visible and r.key.team != team]
        sign = world.direction(team)

        def options(anchor, side):
            for advance, spread in ((0, 0), (0, 0.42), (-0.58, 0.22), (0.38, -0.32)):
                x = max(-4.30, min(4.30, anchor[0] + sign * advance))
                y = max(-2.73, min(2.73, anchor[1] + sign * side * spread))
                gap = clearance(ball.x, ball.y, x, y, rivals, 2.3)
                space = min((math.hypot(q.x - x, q.y - y) for q in rivals), default=1.0)
                value = min(0.65, gap) + 0.35 * min(1.0, space) - 0.22 * math.hypot(advance, spread)
                yield (x, y), value

        def pair_value(pair):
            (a, va), (b, vb) = pair
            # Reward outlets on different bearings, so one defender cannot
            # block both with the same move.
            separation = math.dist(a, b) / max(
                1.0, math.dist(a, (ball.x, ball.y)) + math.dist(b, (ball.x, ball.y))
            )
            return va + vb + 0.24 * separation

        a, b = max(product(options(left, 1), options(right, -1)), key=pair_value)
        return a[0], b[0]

    def attack(self, world, carrier, strategy, weights, keeper):
        team = carrier.key.team
        ball, sign = world.ball, world.direction(team)
        others = [r for r in world.robots if r.visible and r.key != carrier.key]
        rivals = [r for r in others if r.key.team != team]
        gx, gy, gap = shot_window(ball.x, ball.y, sign, others)
        if sign * (gx - ball.x) < strategy.book.shoot_range and gap > 0.13:
            return AttackDecision(Intent(Skill.SHOOT, (gx, gy)), "An open shot is within range.")
        pressure = min((math.hypot(q.x - ball.x, q.y - ball.y) for q in rivals), default=math.inf)
        options = []
        for score, mate, progress in pass_options(world, carrier, weights):
            if not mate.visible or not strategy.pass_is_available(keeper, pressure, progress):
                continue
            target = pass_target(mate)
            blockers = [q for q in others if q.key != mate.key]
            clearance_m = clearance(ball.x, ball.y, *target, blockers, 2.3)
            space = min(
                (
                    math.hypot(q.x + 0.4 * q.vx - target[0], q.y + 0.4 * q.vy - target[1])
                    for q in rivals
                ),
                default=1.0,
            )
            if clearance_m >= 0.20 and space >= 0.32:
                value = score + 1.4 * min(0.55, clearance_m) + 0.4 * min(0.8, space)
                options.append((value, mate.key, target))
        if options:
            _, receiver, target = max(options, key=lambda row: row[0])
            jersey = f"{receiver.team[0].upper()}{receiver.id}"
            return AttackDecision(
                Intent(Skill.PASS, target, receiver), f"{jersey} has an open passing lane."
            )
        return AttackDecision(Intent(Skill.DRIBBLE, (gx, gy)), "Carry while the outlets clear.")


CONTROLLERS: dict[str, TeamController] = {
    "balanced": BalancedController(),
    "priority": LocalPriorityController(),
    "coordinated": CoordinatedController(),
}
