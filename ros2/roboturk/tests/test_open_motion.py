from dataclasses import replace
from math import hypot

import pytest

from roboturk.five_play import crowded_chaser
from roboturk.five_roles import Assignment, CFG, FiveRoles
from roboturk.model import Ball, Command, Robot, RobotKey, World
from roboturk.open_motion import FETCH_SPEED, Steering, observed_velocity, steer_team


def test_practice_steering_uses_the_speed_limit_when_the_lane_is_clear():
    runner = Robot(RobotKey("blue", 2), 0, 0)
    rival = Robot(RobotKey("yellow", 3), 0.7, 0)
    policy = Steering(0.48, 0.305, 0.66)
    cfg = replace(CFG, max_speed=4, max_acceleration=10)

    def drive(robots, practice):
        commands = steer_team(
            robots,
            "blue",
            {runner.key: Command(4, 0)},
            {r.key: (0, 0) for r in robots},
            policy,
            cfg.dt,
            cfg=cfg,
            expand_pace=practice,
        )
        return commands[runner.key]

    assert drive((runner,), False).vx == pytest.approx(1.2)
    assert drive((runner,), True).vx == pytest.approx(4)
    detour = drive((runner, rival), True)
    assert detour.vx < 4
    assert hypot(detour.vx, detour.vy) <= 4


def test_opponent_input_is_causal_position_history_only():
    blue = Robot(RobotKey("blue", 2), 0, 0)
    yellow = Robot(RobotKey("yellow", 3), 0.50, 0, vx=10)
    robots = (blue, yellow)
    measured = observed_velocity(robots, {yellow.key: (0.54, 0)}, 0.04)
    assert measured[yellow.key] == pytest.approx((-1, 0))
    cmd = Command(1, 0, 0.5, 1, 2.6)
    req = {blue.key: cmd}
    actor = Steering(0.38, 0.30, 0.60)

    def steer(roster, requests):
        return steer_team(roster, "blue", requests, measured, actor, 0.04)[blue.key]

    sent = steer(robots, req)
    assert sent != cmd and (sent.omega, sent.dribbler, sent.kick) == (0.5, 1, 2.6)
    assert sent == steer((blue, replace(yellow, vx=-10, vy=123, yaw=2)), req)
    with pytest.raises(ValueError):
        steer(robots, {**req, yellow.key: Command(-1, 0, 0)})
    assert all(v == (0, 0) for v in observed_velocity(robots, {}, 0.04).values())


def test_keeper_mouth_access_requires_clearance_from_the_posts():
    keeper = Robot(RobotKey("blue", 0), -4.35, 0.0)
    policy = Steering(0.48, 0.305, 0.66)
    request = {keeper.key: Command(-0.4, 0, 0, 1)}

    def drive(robot, active):
        return steer_team(
            (robot,),
            "blue",
            request,
            {robot.key: (0, 0)},
            policy,
            0.04,
            goal_cover={robot.key: -1} if active else None,
        )[robot.key]

    assert drive(keeper, True).vx < 0
    assert drive(keeper, False).vx == 0
    assert drive(replace(keeper, y=0.40), True).vx == 0
    assert drive(replace(keeper, y=0.32, vy=0.6), True).vx == 0


def test_handed_off_keeper_can_approach_quickly_and_still_brakes_at_roller_range():
    keeper = Robot(RobotKey("blue", 0), 0, 0)
    presser = Robot(RobotKey("blue", 1), 0, 1)
    rival = Robot(RobotKey("yellow", 1), 0.76, 0)
    policy = Steering(0.48, 0.305, 0.66)
    observed = {r.key: (0, 0) for r in (keeper, presser, rival)}
    commands = {keeper.key: Command(1.16, 0, 0, 1), presser.key: Command()}

    def approach(other, timed):
        return steer_team(
            (keeper, presser, other),
            "blue",
            commands,
            observed,
            policy,
            0.04,
            challenges={keeper.key: other},
            keeper_handoffs={keeper.key: presser.key} if timed else None,
        )[keeper.key].vx

    assert approach(rival, timed=False) == pytest.approx(0.32)
    assert approach(rival, timed=True) > 0.9
    assert approach(replace(rival, x=0.202), timed=True) < 0.13


def test_loose_ball_chaser_has_a_close_approach_and_brakes_before_touching():
    blue = Robot(RobotKey("blue", 2), 0, 0)
    yellow = Robot(RobotKey("yellow", 3), 0.305, 0)
    actor = Steering(0.48, 0.305, 0.66)
    request = {blue.key: Command(0.5, 0, dribbler=1)}

    def approach(rival, close, incoming=0):
        return steer_team(
            (blue, rival),
            "blue",
            request,
            {blue.key: (0, 0), rival.key: (incoming, 0)},
            actor,
            0.04,
            fetching={blue.key} if close else (),
        )[blue.key]

    assert approach(yellow, False).vx < 0.16
    approach_command = approach(yellow, True)
    assert approach_command.vx > 0.20
    assert approach_command.dribbler == 1
    assert approach_command.vx <= FETCH_SPEED
    assert approach(replace(yellow, x=0.202), True).vx <= 1e-8
    assert approach(replace(yellow, x=0.23), True, incoming=-0.6).vx <= 1e-8


def test_crowded_chasers_wait_for_a_stall_then_keep_one_fetcher_until_recovery():
    blue = Robot(RobotKey("blue", 2), -0.16, 0)
    yellow = Robot(RobotKey("yellow", 3), 0.18, 0)
    world = World(0, (blue, yellow), Ball(0, 0))
    roles = {r.key: Assignment("Chaser", (0, 0)) for r in world.robots}
    tactics = FiveRoles()
    actor = Steering(0.48, 0.305, 0.66)
    observed = {r.key: (0, 0) for r in world.robots}

    for _ in range(4):
        assert not crowded_chaser(world, tactics, roles, observed, actor, (), None)

    assert crowded_chaser(world, tactics, roles, observed, actor, (), None) == {blue.key}
    adjusted = replace(world, robots=(blue, replace(yellow, x=0.15)))
    assert crowded_chaser(adjusted, tactics, roles, observed, actor, (), None) == {blue.key}
    tactics.control = blue.key
    assert not crowded_chaser(adjusted, tactics, roles, observed, actor, (), blue.key)
    assert not tactics.fetch_stalls
