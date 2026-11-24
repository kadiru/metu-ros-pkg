from dataclasses import replace
import math

from roboturk.errt import ERRTFallback, edge_clear, opens_clearance
from roboturk.five_roles import Assignment
from roboturk.model import Command, Robot, RobotKey
from roboturk.timed_motion import INTERPOLATION_MARGIN, TIMES, choose, clear, plan_team


def test_blocked_detours_get_a_seeded_time_checked_route_and_reuse_its_cache():
    r = Robot(RobotKey("blue", 1), 0, 0)
    target = (2, 0)
    tracks = [[(0.39, 0)] * len(TIMES)]
    assert choose(r, target, tracks, 0.305)[2] == "brake"
    planner = ERRTFallback(42)
    result = planner.search(r, target, "Outlet", tracks, 0.305)
    assert result and clear(result[1], tracks, 0.305)
    assert ERRTFallback(42).search(r, target, "Outlet", tracks, 0.305) == result
    memo = planner.memory[r.key]
    state = memo.rng.getstate()
    assert planner.search(r, target, "Outlet", tracks, 0.305) == result
    assert memo.rng.getstate() == state
    # A moving obstacle crosses the cached motion after the initial observation.
    endpoint = result[1][-1]
    crossing = [(endpoint[0], endpoint[1] + 1.2 - t) for t in TIMES]
    blocked = tracks + [crossing]
    assert not clear(result[1], blocked, 0.305)
    alternate = planner.search(r, target, "Outlet", blocked, 0.305)
    assert alternate is None or clear(alternate[1], blocked, 0.305)
    planner.search(r, (-2, 0), "Cover", [], 0.305)
    assert planner.memory[r.key] is not memo


def test_fallback_only_after_brake_and_motion_information_is_team_local():
    r = Robot(RobotKey("yellow", 1), 0, 0)
    q = Robot(RobotKey("blue", 3), 0.39, 0)
    goal = (2, 0)
    req = {r.key: Command(omega=1, dribbler=1)}
    roles = {r.key: Assignment("Outlet", goal)}

    def run(opponent):
        counts = {}
        planner = ERRTFallback(42)
        out = plan_team(
            (r, opponent),
            "yellow",
            req,
            {r.key: goal},
            roles,
            {q.key: (0, 0)},
            0.305,
            counts,
            planner,
        )
        assert (out[r.key].omega, out[r.key].dribbler) == (1, 1)
        return counts, out, planner.memory

    counts, out, _ = run(q)
    assert counts == {"errt": 1}
    assert run(replace(q, vx=-12, vy=10, yaw=1.2))[1] == out
    for pos, expected in (((0.60, 0), "detour"), ((0, 2), "direct")):
        counts, _, cache = run(replace(q, x=pos[0], y=pos[1]))
        assert counts == {expected: 1} and not cache
    counts, _, _ = run(replace(q, x=0.1))
    assert counts == {"brake": 1}


def test_geometric_search_respects_exclusions_and_sweeps_through_disks():
    assert not edge_clear((3.2, 0.7), (3.8, 1.3), [], 0.3)
    assert not edge_clear((0, 2.6), (0, 3.02), [], 0.3)
    assert not edge_clear((-1, 0), (1, 0), [(0, 0.2)], 0.3)
    assert edge_clear((-1, 0), (1, 0), [(0, 0.4)], 0.3)


def test_hold_survives_a_new_detour_but_releases_when_blocked_reassigned_or_reached():
    r = Robot(RobotKey("blue", 1), 0, 0)
    target = (2, 0)
    q = Robot(RobotKey("yellow", 1), 0.39, 0)
    planner = ERRTFallback(42, commit_waypoints=True)

    def tick(pos, role="Outlet", targets=None):
        modes = {}
        plan_team(
            (r, replace(q, x=pos)),
            "blue",
            {r.key: Command()},
            {r.key: target} if targets is None else targets,
            {r.key: Assignment(role, target)},
            {q.key: (0, 0)},
            0.305,
            fallback=planner,
            modes=modes,
        )
        return modes.get(r.key)

    assert tick(0.39) == "errt"
    point = planner.memory[r.key].committed
    assert choose(r, target, [[(0.60, 0)] * len(TIMES)], 0.305)[2] == "detour"
    assert tick(0.60) == "hold" and planner.memory[r.key].committed == point
    crossing = [(point[0], point[1] + 1.2 - t) for t in TIMES]
    assert planner.resume(r, target, "Outlet", [crossing], 0.305) is None
    assert planner.memory[r.key].committed is None
    assert tick(0.39) == "errt"
    assert tick(0.60, "Cover") == "detour"
    assert tick(0.39) == "errt"
    assert planner.resume(r, (-2, 0), "Outlet", [], 0.305) is None
    assert tick(0.39) == "errt"
    point = planner.memory[r.key].committed
    assert planner.resume(replace(r, x=point[0], y=point[1]), target, "Outlet", [], 0.305) is None
    assert tick(0.39) == "errt"
    assert tick(0.39, targets={}) is None and r.key not in planner.memory


def test_outward_waypoint_restores_the_buffer_and_checks_all_obstacles():
    r = Robot(RobotKey("yellow", 1), 0, 0)
    close = [(-0.27, 0)] * len(TIMES)
    planner = ERRTFallback(1, commit_waypoints=True)
    out = planner.search(r, (2, 0), "Outlet", [close], 0.305)
    path = out[1]
    assert planner.memory[r.key].restoring and opens_clearance(path, [close], 0.305)
    assert math.dist(path[-1], close[-1]) > 0.305
    assert not opens_clearance(path[::-1], [close], 0.305)
    assert not opens_clearance(path, [close, [path[-1]] * len(TIMES)], 0.305)
    assert not opens_clearance(path, [[(-0.18, 0)] * len(TIMES)], 0.305)
    assert not opens_clearance([(0, 0)] * len(TIMES), [close], 0.305)
    assert planner.search(replace(r, vx=0.25), (2, 0), "Outlet", [close], 0.305) is None
    assert ERRTFallback(1).search(r, (2, 0), "Outlet", [close], 0.305) is None


def test_selective_commitment_requires_a_return_with_little_progress():
    r = Robot(RobotKey("blue", 1), 0, 0)
    q = Robot(RobotKey("yellow", 2), 0.39, 0)
    goal = (2, 0)

    def scenario(x):
        planner = ERRTFallback(42, commit_waypoints="on-conflict")

        def tick(robot, opponent):
            modes = {}
            plan_team(
                (robot, opponent),
                "blue",
                {r.key: Command()},
                {r.key: goal},
                {r.key: Assignment("Outlet", goal)},
                {q.key: (0, 0)},
                0.305,
                fallback=planner,
                modes=modes,
            )
            return modes[r.key]

        assert tick(r, q) == tick(r, q) == "errt"
        assert planner.memory[r.key].committed is None
        assert tick(r, replace(q, x=0.6)) == "detour"
        assert tick(replace(r, x=x), replace(q, x=x + 0.39)) == "errt"
        return planner.memory[r.key].committed

    assert scenario(0.02) is not None
    assert scenario(0.20) is None


def test_stopped_runner_can_restore_clearance_inside_the_interpolation_margin():
    r = Robot(RobotKey("yellow", 1), 0, 0)
    target = (2, 0)
    clearance = 0.305
    gap = clearance + INTERPOLATION_MARGIN / 2
    tracks = [[(gap, 0)] * len(TIMES)]
    assert ERRTFallback(1).plan(r, target, "Outlet", tracks, clearance)[2] == "brake"
    planner = ERRTFallback(1, commit_waypoints="on-conflict")
    motion, path, mode = planner.plan(r, target, "Outlet", tracks, clearance)
    assert mode == "escape" and math.hypot(*motion) > 0
    assert opens_clearance(path, tracks, clearance)
    assert math.dist(path[-1], tracks[0][-1]) >= clearance + INTERPOLATION_MARGIN
    blocked = [*tracks, [(-0.18, 0)] * len(TIMES)]
    assert (
        ERRTFallback(1, commit_waypoints="on-conflict").plan(
            r, target, "Outlet", blocked, clearance
        )[2]
        == "brake"
    )


def test_early_release_requires_four_clear_updates_and_keeps_outward_recovery():
    robot = Robot(RobotKey("blue", 1), 0, 0)
    target, waypoint = (2, 0), (0, -1)
    planner = ERRTFallback(42, commit_waypoints="on-conflict", early_release=True)
    # Seed a held waypoint to focus this test on the replanning decision.
    planner.search(robot, target, "Outlet", [[(0.39, 0)] * len(TIMES)], 0.305)
    memo = planner.memory[robot.key]
    memo.committed = waypoint
    memo.waypoints = (waypoint,)

    def mode(tracks=()):
        return planner.plan(robot, target, "Outlet", tracks, 0.305)[2]

    assert [mode() for _ in range(3)] == ["hold"] * 3
    barely_clear = [[(0.6, 0.32)] * len(TIMES)]
    assert choose(robot, target, barely_clear, 0.305)[2] == "direct"
    assert mode(barely_clear) == "hold"
    assert [mode() for _ in range(3)] == ["hold"] * 3
    assert mode() == "shortcut"
    assert memo.committed is None
    assert mode() == "direct"

    memo.committed = waypoint
    memo.restoring = True
    assert [mode() for _ in range(5)] == ["hold"] * 5
