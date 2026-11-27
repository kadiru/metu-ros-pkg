import math
from dataclasses import replace

import pytest

from roboturk.model import Ball, Command, Robot, world_vector
from roboturk.primitive_dojo import (
    ACTOR,
    CFG,
    RECEIVER,
    SERVERS,
    TASKS,
    Arrival,
    ArrivalRoute,
    deflection_station,
    go_to,
    ground_prediction,
    intercept,
    pass_plan,
    scenario,
    serve_relay,
    stop_at,
)
from roboturk.wheel_drive import (
    WHEEL_ANGLES_DEG,
    beam_broken,
    limit_by_shafts,
    shafts_for_twist,
    twist_for_shafts,
)


def test_terminal_velocity_and_aim_are_independent():
    target = Arrival.at_speed((1, 2), -math.pi / 2, 1.1, travel_heading=0)
    robot = Robot(ACTOR, 1, 2, -math.pi / 2)
    command = go_to(robot, target)
    assert world_vector(command.vx, command.vy, robot.yaw) == pytest.approx((1.1, 0))
    assert command.omega == 0
    assert go_to(robot, target, terminal=False) == Command()


def test_stationary_arrival_reserves_braking_distance_and_damps_lateral_motion():
    robot = Robot(ACTOR, 0, 0, vx=0.6, vy=0.4)
    goal = Arrival((0.064, 0), 0)
    command = stop_at(robot, goal)
    assert command.vy < 0
    brake = 0.7 * CFG.max_acceleration
    assert command.vx * CFG.dt + command.vx**2 / (2 * brake) == pytest.approx(0.06)
    assert stop_at(robot, Arrival((0.002, 0), 0)) == Command()
    cruise = stop_at(robot, Arrival((3, 0), 0))
    assert math.hypot(cruise.vx, cruise.vy) == pytest.approx(2.4)
    with pytest.raises(ValueError, match="zero terminal velocity"):
        stop_at(robot, Arrival.at_speed((1, 0), 0, 1))


def test_off_axis_arrival_moves_behind_target_before_final_approach():
    goal = Arrival.at_speed((0, 0), math.pi / 2, 1.2, travel_heading=0)
    route = ArrivalRoute.from_pose(Robot(ACTOR, 0.2, 0.9), goal)
    assert route.entry is not None
    assert route.entry.point[0] < 0
    entry_pose = Robot(ACTOR, *route.entry.point, goal.yaw)
    command = route.command(entry_pose)
    assert route.entry is None
    assert world_vector(command.vx, command.vy, entry_pose.yaw)[0] > 0


@pytest.mark.parametrize("task", TASKS)
def test_seeded_drills_repeat_and_cover_both_contexts(task):
    assert scenario(task, 12600) == scenario(task, 12600)
    assert scenario(task, 12600).context != scenario(task, 12601).context


def test_rolling_contact_prediction_stops_and_cannot_reverse():
    ball = Ball(0, 0, 1.2, 0)
    far, velocity = ground_prediction(ball, 50)
    repeat, _ = ground_prediction(ball, 500)
    assert velocity == pytest.approx((0, 0))
    assert far == pytest.approx(repeat)
    assert far[0] > 0


def test_body_redirect_leaves_both_ball_actuators_off():
    drill = scenario("intercept", 12601)
    r = drill.world.robot(ACTOR)
    assert drill.outlet is not None
    command = intercept(drill.world, r, {"predict": True}, drill.outlet)
    assert command.dribbler == command.kick == 0
    assert CFG.dt == 0.04


def test_relay_arms_the_kicker_only_when_ball_arrives_at_the_next_server():
    drill = scenario("intercept", 12600)
    a, b = SERVERS
    assert serve_relay(drill.world, a, drill.serve_speed)[a].kick == drill.serve_speed
    assert serve_relay(drill.world, b, drill.serve_speed)[b].kick == 0
    receiver = drill.world.robot(b)
    ball = Ball(receiver.x + CFG.possession_x * math.cos(receiver.yaw), receiver.y)
    arrived = replace(drill.world, ball=ball)
    assert serve_relay(arrived, b, drill.serve_speed)[b].kick == drill.serve_speed


def test_planned_redirect_places_chassis_on_the_opposite_side_from_outlet():
    drill = scenario("intercept", 12601)
    world = replace(drill.world, ball=Ball(-1.6, 0, 1.85, 0))
    station = deflection_station(
        world, world.robot(ACTOR), (0.8, -1.45), speed=1.65, settle=0.28, restitution=0.2
    )
    assert station.point[0] > world.ball.x
    assert station.point[1] > 0
    assert station.velocity == (0, 0)


def test_lead_target_moves_with_teammate_velocity():
    drill = scenario("pass", 12601)
    mate = drill.world.robot(RECEIVER)
    feet, _ = pass_plan(drill.world.ball, mate, {})
    lead, power = pass_plan(drill.world.ball, mate, {"lead": 1})
    assert feet == (mate.x, mate.y)
    assert (lead[1] - mate.y) * mate.vy > 0
    assert 0 < power <= CFG.max_kick


def test_all_four_wheels_share_one_saturation_factor():
    assert WHEEL_ANGLES_DEG == (60, 135, -135, -60)
    command = Command(2.1, -1.2, 5, 1, 3, 0.1)
    request = command.vx, command.vy, command.omega
    assert twist_for_shafts(shafts_for_twist(*request)) == pytest.approx(request)
    limited = limit_by_shafts(command, 90)
    assert max(map(abs, shafts_for_twist(limited.vx, limited.vy, limited.omega))) == pytest.approx(
        90
    )
    assert limited.vx / command.vx == pytest.approx(limited.omega / command.omega)
    assert (limited.dribbler, limited.kick, limited.kick_elevation) == (1, 3, 0.1)


def test_ball_sensor_tracks_front_roller_in_robot_frame_and_ignores_airborne_ball():
    r = Robot(ACTOR, 0, 0, math.pi / 2)
    assert beam_broken(r, Ball(0, CFG.possession_x), CFG.ball_radius)
    assert not beam_broken(r, Ball(CFG.possession_x, 0), CFG.ball_radius)
    assert not beam_broken(r, Ball(0, CFG.possession_x, z=0.1), CFG.ball_radius)
    assert not beam_broken(r, Ball(0, CFG.possession_x, visible=False), CFG.ball_radius)
