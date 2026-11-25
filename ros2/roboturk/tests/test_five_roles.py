from dataclasses import replace
import math
import pytest
from roboturk.five_roles import (
    OFF_BALL_ROLES,
    ROLE_SWITCH_COST,
    TEAMS,
    FiveRoles,
    Flight,
    kickoff,
    legal_off_ball_target,
)
from roboturk.model import Ball, RobotKey
from roboturk.timed_motion import clear, inside


def test_ten_roles_and_goalkeeper_identity():
    world = kickoff(12, "blue", 1)
    tactics = FiveRoles()
    for _ in range(3):
        owner = tactics.observe(world)
    for team in TEAMS:
        phase, roles = tactics.assign(world, team)
        assert phase == ("Attack" if team == "blue" else "Defend")
        assert len(roles) == len({a.role for a in roles.values()}) == 5
        assert roles[RobotKey(team, 0)].role == "Keeper"
        if team == "blue":
            assert roles[owner].role == "Carrier"


def test_pass_recipient_is_bound_until_opponent_gains_control():
    w = kickoff(15, "yellow", -1)
    t = FiveRoles()
    for _ in range(3):
        holder = t.observe(w)
    receiver = next(r.key for r in w.robots if r.key.team == "blue" and r.key.id != 0)
    t.flight = Flight("blue", receiver, (1.0, -0.6), w.time + 1)
    phase, roles = t.assign(w, "blue")
    assert phase == "Attack" and roles[receiver].role == "Receiver"
    t.observe(w)
    assert t.flight is None and t.control == holder
    assert t.assign(w, "blue")[0] == "Defend"


def test_audit_records_the_pin_and_exact_winning_roster_cost():
    world = kickoff(9, "blue", 1)
    receiver = RobotKey("blue", 2)
    tactics = FiveRoles()
    tactics.flight = Flight("blue", receiver, (-0.3, 0.5), 1.65)
    tactics.previous = {r.key: "Chaser" for r in world.robots if r.key.team == "blue" and r.key.id}
    previous = tactics.previous.copy()
    baseline = tactics.assign(world, "blue")
    tactics.previous = previous
    evidence = {}
    phase, roles = tactics.assign(world, "blue", audit=evidence)
    assert (phase, roles) == baseline
    assert evidence[receiver]["basis"] == "receiver"
    assert evidence[RobotKey("blue", 0)]["basis"] == "keeper"
    total = 0.0
    for key, assignment in roles.items():
        if key.id:
            robot = world.robot(key)
            distance = math.dist((robot.x, robot.y), assignment.target)
            total += distance + ROLE_SWITCH_COST
            assert evidence[key]["distance_m"] == round(distance, 4)
            assert evidence[key]["allocator_previous"] == "Chaser"
            assert evidence[key]["switch_cost"] == ROLE_SWITCH_COST
    assert evidence[receiver]["roster_cost"] == round(total, 4)


def test_assignment_moves_with_the_players_and_respects_attack_direction():
    w = kickoff(9, "blue", 1)
    a = FiveRoles()
    _, first = a.assign(w, "blue")
    keys = [k for k in first if k.id]
    swapped = {keys[0]: keys[1], keys[1]: keys[0]}
    alt = replace(
        w,
        robots=tuple(replace(r, key=swapped.get(r.key, r.key), x=-r.x, y=-r.y) for r in w.robots),
        ball=replace(w.ball, x=-w.ball.x, y=-w.ball.y),
        attack_sign=-1,
    )
    _, second = FiveRoles().assign(alt, "blue")
    for key, role in first.items():
        other = second[swapped.get(key, key)]
        assert other.role == role.role and other.target == tuple(-v for v in role.target)


def test_off_ball_targets_clear_the_buffer_on_both_penalty_areas():
    for side in (-1, 1):
        front = legal_off_ball_target((side * 3.47, -0.31))
        flank = legal_off_ball_target((side * 4.1, -1.0))
        assert front == pytest.approx((side * 3.382, -0.31))
        assert flank == pytest.approx((side * 4.1, -1.118))
        for point in (front, flank):
            assert inside(point) and clear([point, point], [], 0)
        legal = (side * 3.9, 1.25)
        assert legal_off_ball_target(legal) == legal


def test_cover_targets_are_corrected_before_role_assignment_and_keeper_can_enter():
    for sign in (-1, 1):
        world = replace(kickoff(0, "blue", sign), ball=Ball(-3.08 * sign, -0.73 * sign))
        for team in TEAMS:
            _, roles = FiveRoles().assign(world, team)
            assert not inside(roles[RobotKey(team, 0)].target)
            for assignment in roles.values():
                if assignment.role in OFF_BALL_ROLES:
                    assert clear([assignment.target, assignment.target], [], 0)
        _, released = FiveRoles().assign(world, "blue", legal_off_ball=False)
        _, corrected = FiveRoles().assign(world, "blue")
        for roster, legal in [(released, False), (corrected, True)]:
            target = next(a.target for a in roster.values() if a.role == "Cover")
            assert inside(target) is legal
