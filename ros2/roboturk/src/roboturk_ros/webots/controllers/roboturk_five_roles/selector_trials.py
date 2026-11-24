"""Counterfactual tackle choices from identical native shared-roller starts."""

from dataclasses import asdict, replace
import math
import json
import random

from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.five_roles import CFG, TEAMS, kickoff
from roboturk.model import Ball, Command, RobotKey, possession
from roboturk.open_motion import Steering, observed_velocity, steer_team
from roboturk.pivot_fetch import SWEEP_RADIANS, SWEEP_SECONDS, fetch_loose, sweep_command
from roboturk.tackle import BallSideTackle
from roboturk.tackle_selector import ACTIONS, CONTAIN, PIVOT_SIDES, TackleSelector, contain_command

SECONDS = 1.80
PANEL_ACTIONS = (*ACTIONS, "selector")


def opening(zone, seed):
    rng = random.Random(seed)
    team = TEAMS[(seed // 2) % 2]
    presser = RobotKey(team, 1)
    carrier = RobotKey(TEAMS[1 - TEAMS.index(team)], 1)
    attack = 1 if (seed // 4) % 2 == 0 else -1
    reflection = 1 if seed % 2 == 0 else -1
    world = kickoff(seed, carrier.team, attack)
    sign = world.direction(team)
    spread = rng.uniform(-0.18, 0.18)
    near_line = (seed // 4) % 2 == 0
    vx = vy = 0.0
    if zone == "touchline":
        x = rng.uniform(-1.4, 1.4)
        inset = rng.uniform(0.001, 0.025) if near_line else rng.uniform(0.08, 0.16)
        y = reflection * (CFG.width / 2 - inset)
        vy = reflection * 0.045 if seed % 4 in (0, 3) else 0.0
        yaw = (0 if sign > 0 else math.pi) + reflection * spread
    elif zone == "own-goal":
        inset = rng.uniform(0.025, 0.085) if near_line else rng.uniform(0.13, 0.28)
        x = -sign * (CFG.length / 2 - inset)
        vx = -sign * (0.13 if seed % 4 in (0, 3) else 0.03)
        y = reflection * rng.uniform(0.12, 0.46)
        yaw = (math.pi if sign > 0 else 0) if seed % 4 < 2 else -reflection * math.pi / 2
        yaw += reflection * spread
    else:
        x, y = rng.uniform(-0.75, 0.75), rng.uniform(-0.55, 0.55)
        yaw = rng.uniform(-math.pi, math.pi)
    second = yaw + math.pi + rng.uniform(-0.20, 0.20)
    poses = {
        carrier: (x - 0.115 * math.cos(yaw), y - 0.115 * math.sin(yaw), yaw),
        presser: (x - 0.121 * math.cos(second), y - 0.121 * math.sin(second), second),
    }
    parked = {0: (3.10, 1.15), 2: (2.75, -1.2), 3: (2.68, 2.2), 4: (1.93, -2.05)}
    roster = []
    for r in world.robots:
        sx, sy = parked.get(r.key.id, (0, 0))
        team_sign = 1 if r.key.team == "blue" else -1
        rx, ry, angle = poses.get(r.key, (sx * team_sign, sy * team_sign, r.yaw))
        roster.append(replace(r, x=rx, y=ry, yaw=angle))
    return replace(world, ball=Ball(x, y, vx, vy), robots=tuple(roster)), carrier, presser


def hold_carrier(seed, time):
    mode = seed % 4
    side = 1 if seed // 4 % 2 else -1
    return Command(
        vx=(0, -0.13, 0.08, 0.10)[mode],
        vy=side * 0.10 * math.sin(3.2 * time) if mode == 2 else 0,
        omega=side * 0.42 if mode == 3 else 0,
        dribbler=1,
    )


def toward_own_goal(ball, sign):
    distance = CFG.length / 2 + sign * ball.x
    speed = -sign * ball.vx
    return (
        0 <= distance <= 0.65
        and speed >= 0.10
        and abs(ball.y + distance * ball.vy / speed) < CFG.goal_width / 2
    )


def trial(backend, policy, profile, start, carrier, presser, seed, zone, mode):
    world = backend.reset(start)
    assert all(possession(world, key, CFG) for key in (carrier, presser))
    initial = asdict(world)
    selector = TackleSelector(profile) if mode == "selector" else None
    gap = 10.0
    contact = False
    goalward = out = own_goal = capture = last_sweep = None
    dwell = 0
    previous = {}
    yaw_target = world.robot(presser).yaw - PIVOT_SIDES.get(mode, 0) * SWEEP_RADIANS
    sign = world.direction(presser.team)
    physics_dt = backend.basic_ms / 1000
    confirmation = round(0.24 / physics_dt)
    for _ in range(round(SECONDS / CFG.dt)):
        observed = observed_velocity(world.robots, previous, CFG.dt)
        previous = {r.key: (r.x, r.y) for r in world.robots}
        player, rival = world.robot(presser), world.robot(carrier)
        has_ball = possession(world, presser, CFG)
        rival_holds = possession(world, carrier, CFG)
        action = mode
        if mode == CONTAIN:
            request = contain_command(world, player)
        elif has_ball and not rival_holds:
            # All release choices use the same stationary hold for the follow-through.
            request = Command(dribbler=1)
        elif selector:
            fetch = selector.collect_loose(world, player)
            request = fetch or selector.command(world, player, rival, observed, has_ball)
            action = "collect" if fetch else selector.action(presser)
        elif mode in PIVOT_SIDES and world.time < SWEEP_SECONDS and has_ball and rival_holds:
            last_sweep = world.time
            request = sweep_command(world, player, rival, PIVOT_SIDES[mode], yaw_target)
        else:
            fetch = fetch_loose(world, player, last_sweep)
            request = fetch or profile.command(world, player, rival, observed, has_ball)
        players = {r.key: Command() for r in world.robots if r.key.team == presser.team}
        players[presser] = request
        commands = steer_team(
            world.robots,
            presser.team,
            players,
            observed,
            policy,
            CFG.dt,
            # Touchline cases use the same board-aware envelope as live touchline
            # recovery. The setup can put the dribblers over the painted line.
            recovering=(presser,) if zone == "touchline" else (),
            challenges={presser: rival} if action != CONTAIN and rival_holds else None,
        )
        commands[carrier] = hold_carrier(seed, world.time)
        world = backend.step(commands)
        for state in backend.last_substep_worlds:
            b = state.ball
            r = state.robot(presser)
            gap = min(
                gap,
                min(
                    math.hypot(r.x - q.x, r.y - q.y) - 2 * CFG.robot_radius
                    for q in state.robots
                    if r.key != q.key
                ),
            )
            contact |= gap <= 0.001
            received = request.dribbler > 0.5 and possession(state, presser, CFG)
            held = possession(state, carrier, CFG)
            dwell = dwell + 1 if received and not held else 0
            if dwell >= confirmation and capture is None:
                capture = state.time
            if not received and not held and goalward is None and toward_own_goal(b, sign):
                goalward = state.time
            through_end = abs(b.x) > CFG.ball_exit_x
            if through_end or abs(b.y) > CFG.ball_exit_y:
                if through_end and abs(b.y) < CFG.goal_width / 2 - CFG.ball_radius:
                    if sign * b.x < 0:
                        own_goal = state.time
                else:
                    out = state.time
                break
        if through_end or out is not None:
            break
    return dict(
        initial=initial,
        capture_s=capture,
        retained=dwell >= confirmation,
        clean_retained=dwell >= confirmation
        and not contact
        and out is None
        and own_goal is None
        and goalward is None,
        body_contact=contact,
        min_gap_m=gap,
        ball_out_s=out,
        own_goal_s=own_goal,
        goalward_release_s=goalward,
        decisions=[
            dict(e, presser=asdict(e["presser"]), carrier=asdict(e["carrier"]))
            for e in selector.events
        ]
        if selector
        else [],
    )


def run(sim, folder, cfg):
    policy = Steering(**cfg["open_play_policy"])
    profile = BallSideTackle(**cfg["tackle_policy"])
    backend = ContinuousWebotsBackend(sim, CFG)
    zone = cfg["tackle_panel"]
    output = []
    for seed in range(cfg["seed"], cfg["seed"] + cfg["tackle_trials"]):
        start, carrier, presser = opening(zone, seed)
        outcomes = {
            mode: trial(backend, policy, profile, start, carrier, presser, seed, zone, mode)
            for mode in PANEL_ACTIONS
        }
        initial = outcomes["direct"]["initial"]
        if any(r.pop("initial") != initial for r in outcomes.values()):
            raise RuntimeError("Matched choices started from different native states")
        output.append(dict(zone=zone, seed=seed, initial=initial, outcomes=outcomes))
    (folder / "tackle-panel.json").write_text(json.dumps(output, indent=2) + "\n")
