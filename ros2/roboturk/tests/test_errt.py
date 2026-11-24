from dataclasses import replace

from roboturk.errt import ERRTFallback, edge_clear
from roboturk.five_roles import Assignment
from roboturk.model import Command, Robot, RobotKey
from roboturk.timed_motion import TIMES, choose, clear, plan_team


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
