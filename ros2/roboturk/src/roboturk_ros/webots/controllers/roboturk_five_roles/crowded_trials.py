"""A 5v5 outlet run against a compact marking block, with fixed play assignments."""

from dataclasses import asdict, replace
from itertools import combinations
import json
import math
import random

from roboturk import field
from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.errt import ERRTFallback
from roboturk.five_roles import Assignment, CFG, TEAMS, kickoff
from roboturk.model import Ball, RobotKey
from roboturk.open_motion import Steering, observed_velocity, steer_team
from roboturk.three_recipient import move
from roboturk.timed_motion import plan_team
from touchline_trials import frame_gap


def opening(seed):
    rng = random.Random(seed)
    attack = TEAMS[(seed // 2) % 2]
    sign = 1 if seed % 2 == 0 else -1
    direction = sign if attack == "blue" else -sign
    runner = RobotKey(attack, 1)
    # Rotate and translate the play so the route is not always axis-aligned.
    theta = rng.uniform(-0.29, 0.29)
    cs, sn = math.cos(theta), math.sin(theta)
    cx, cy = rng.uniform(-0.45, 0.45), rng.uniform(-0.60, 0.60)

    def point(u, v):
        return (cx + sign * (cs * u - sn * v), cy + sign * (sn * u + cs * v))

    gap = rng.uniform(0.35, 0.43) if seed % 4 != 3 else rng.uniform(0.54, 0.72)
    stagger = rng.choice((-1, 1))
    run_x = -1.18
    positions = [
        (run_x, 0),
        (-0.95, -1.22 * stagger),
        (-0.73, 1.57 * stagger),
        (-2.25, 0.72 * stagger),
    ]
    defenders = [
        (run_x + gap, rng.uniform(-0.075, 0.075)),
        (-0.28, 0.48 * stagger),
        (0.08, -0.53 * stagger),
        (0.68, 1.05 * stagger),
    ]
    home = {}
    bots = []
    world = kickoff(seed, attack, direction)
    for r in world.robots:
        if r.key.id == 0:
            xy = (r.x, 0)
        else:
            u, v = (positions if r.key.team == attack else defenders)[r.key.id - 1]
            xy = point(u, v)
        home[r.key] = xy
        bots.append(replace(r, x=xy[0], y=xy[1], yaw=theta + (0 if sign > 0 else math.pi)))
    carrier = next(r for r in bots if r.key == RobotKey(attack, 2))
    ball = Ball(
        carrier.x + CFG.possession_x * math.cos(carrier.yaw),
        carrier.y + CFG.possession_x * math.sin(carrier.yaw),
    )
    goal = point(rng.uniform(0.93, 1.38), rng.uniform(-0.16, 0.16))
    return replace(world, robots=tuple(bots), ball=ball), runner, home, goal, stagger


def trial(sim, backend, policy, cfg, seed, frames, fallback):
    initial, runner, home, goal, stagger = opening(seed)
    w = backend.reset(initial)
    initial_state = asdict(w)
    if frames:
        p = sim.getFromDef("CAMERA").getField("position")
        p.setSFVec3f([(w.robot(runner).x + goal[0]) / 2, (w.robot(runner).y + goal[1]) / 2, 6.4])
        # Non-physical marker at the assigned outlet for the recording.
        sim.getRoot().getField("children").importMFNodeFromString(
            -1,
            f"DEF OUTLET_MARKER Transform {{ translation {goal[0]} {goal[1]} 0.003 children [ Shape {{ appearance PBRAppearance {{ baseColor 1 1 1 roughness 1 }} geometry Cylinder {{ radius 0.105 height 0.002 }} }} ] }}",
        )
    previous = {}
    arrival = None
    near = slow = stalls = stall_ticks = 0
    counts = {}
    gaps = dict(robot=20.0, runner=20.0, board=20.0, goal=20.0)
    contact_pairs = set()
    for tick in range(round(cfg["seconds"] / CFG.dt)):
        if frames and tick % 2 == 0:
            sim.exportImage(str(frames / f"{tick // 2:04d}.jpg"), 90)
        measured = observed_velocity(w.robots, previous, CFG.dt)
        previous = {r.key: (r.x, r.y) for r in w.robots}
        out = {}
        for team in TEAMS:
            roles, requests, targets = {}, {}, {}
            for r in w.robots:
                if r.key.team != team:
                    continue
                target = home[r.key]
                role = "Keeper" if r.key.id == 0 else "Marker"
                if r.key == runner:
                    target = goal
                    role = "Outlet"
                elif team == runner.team and r.key.id == 2:
                    role = "Carrier"
                elif r.key.id in (3, 4):
                    # Two supports and two markers shift laterally during the run.
                    amp = 0.26 if team == runner.team else 0.20
                    target = (
                        target[0] + amp * math.sin(w.time * 0.9),
                        target[1]
                        + stagger * amp * math.sin(w.time * 1.2 + r.key.id) * math.sin(w.time),
                    )
                roles[r.key] = Assignment(role, target)
                requests[r.key] = move(r, *target, r.yaw, speed=1.2)
                if role in ("Marker", "Outlet"):
                    targets[r.key] = target
            own = steer_team(w.robots, team, requests, measured, policy, CFG.dt)
            # Both arms keep the same opposition controller.
            out.update(
                plan_team(
                    w.robots,
                    team,
                    own,
                    targets,
                    roles,
                    measured,
                    policy.clearance,
                    counts if team == runner.team else None,
                    fallback=fallback if team == runner.team else None,
                )
            )
        w = backend.step(out)
        r = w.robot(runner)
        distance = math.dist((r.x, r.y), goal)
        if arrival is None:
            near = near + 1 if distance <= 0.10 else 0
            if near == 5:
                arrival = round(w.time, 3)
            slow = slow + 1 if distance > 0.15 and math.hypot(r.vx, r.vy) < 0.08 else 0
            if slow == 15:
                stalls += 1
                stall_ticks += 15
            elif slow > 15:
                stall_ticks += 1
        for state in backend.last_substep_worlds:
            for a, b in combinations(state.robots, 2):
                gap = math.hypot(a.x - b.x, a.y - b.y) - 2 * CFG.robot_radius
                gaps["robot"] = min(gaps["robot"], gap)
                if runner in (a.key, b.key):
                    gaps["runner"] = min(gaps["runner"], gap)
                if gap <= 0.001:
                    contact_pairs.add((a.key, b.key))
            for bot in state.robots:
                rail = (
                    min(
                        field.LENGTH / 2 + field.WALL_MARGIN - abs(bot.x),
                        field.WIDTH / 2 + field.WALL_MARGIN - abs(bot.y),
                    )
                    - CFG.robot_radius
                )
                gaps["board"] = min(gaps["board"], rail)
                gaps["goal"] = min(gaps["goal"], frame_gap(bot))
    if frames:
        sim.getFromDef("OUTLET_MARKER").remove()
    return dict(
        seed=seed,
        initial=initial_state,
        runner=asdict(runner),
        target=goal,
        limit_s=cfg["seconds"],
        arrival_s=arrival,
        stalls=stalls,
        stalled_s=round(stall_ticks * CFG.dt, 3),
        gaps_m=gaps,
        contact_pairs=len(contact_pairs),
        decisions=counts,
    )


def run(sim, folder, cfg):
    backend = ContinuousWebotsBackend(sim, CFG)
    policy = Steering(**cfg["open_play_policy"])
    frames = folder / "frames" if cfg["frame_images"] else None
    if frames:
        frames.mkdir()
    rows = []
    for seed in range(cfg["seed"], cfg["seed"] + cfg["crowded_trials"]):
        fallback = ERRTFallback(seed) if cfg["errt_fallback"] else None
        rows.append(trial(sim, backend, policy, cfg, seed, frames, fallback))
        (folder / "crowded.json").write_text(json.dumps(rows, indent=2) + "\n")
