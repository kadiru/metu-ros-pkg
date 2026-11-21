"""Native contact tests for the continuous-only kicker model, without learned actors."""

import math
from .continuous import KEYS, name
from .model import Ball, Command, Referee, Robot, World


def contact_preflight(backend):
    cases = []
    for swap in (False, True):
        for reverse in (False, True):
            for tie in (False, True):
                left, right = (KEYS[2], KEYS[0]) if swap else (KEYS[0], KEYS[2])
                robots = (Robot(left, -0.115, 0), Robot(right, 0.115 if tie else 0.125, 0, math.pi))
                world = World(0, robots[::-1] if reverse else robots, Ball(0, 0), Referee.RUNNING)
                backend.reset(world)
                backend.step({k: Command(kick=3.5) for k in (left, right)})
                decisions = backend.last_kick_arbitrations
                kicks = backend.last_kick_events
                assert decisions and math.isclose(decisions[0]["time"], 0, abs_tol=1e-8)
                assert decisions[0]["winner"] == (None if tie else left)
                instants = [round(w.time, 8) for _, w, _ in kicks]
                assert len(instants) == len(set(instants))
                if tie:
                    assert not kicks or kicks[0][1].time > 1e-8
                else:
                    assert kicks and kicks[0][0] == left
                cases.append(
                    {
                        "swap_colors": swap,
                        "reverse_iteration": reverse,
                        "tie": tie,
                        "first_winner": name(decisions[0]["winner"])
                        if decisions[0]["winner"]
                        else None,
                        "first_candidates": [[name(k), d] for k, d in decisions[0]["candidates"]],
                        "kicks": [
                            {"robot": name(k), "physics_time": w.time, "speed": v}
                            for k, w, v in kicks
                        ],
                        "final_ball_velocity": [backend.world.ball.vx, backend.world.ball.vy],
                        "passed": True,
                    }
                )
    return {
        "backend": backend.name,
        "backend_contract": backend.contract,
        "control": "scripted_contact_preflight",
        "cases": cases,
        "passed": True,
    }
