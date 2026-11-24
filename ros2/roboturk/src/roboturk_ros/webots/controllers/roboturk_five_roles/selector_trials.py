"""Counterfactual tackle choices from identical native shared-roller starts."""

from dataclasses import asdict, replace
import math
import json
import random

from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.five_roles import CFG, TEAMS, keeper_target, kickoff, limit
from roboturk.model import Ball, Command, RobotKey, possession
from roboturk.open_motion import Steering, observed_velocity, steer_team
from roboturk.pivot_fetch import SWEEP_RADIANS, SWEEP_SECONDS, fetch_loose, sweep_command
from roboturk.tackle import BallSideTackle
from roboturk.tackle_selector import ACTIONS, CONTAIN, PIVOT_SIDES, TackleSelector, contain_command
from roboturk.three_recipient import move

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


def trial(
    backend,
    policy,
    profile,
    start,
    carrier,
    presser,
    seed,
    zone,
    mode,
    keeper_live=False,
    frames=None,
    motion_audit=False,
):
    world = backend.reset(start)
    assert all(possession(world, key, CFG) for key in (carrier, presser))
    initial = asdict(world)
    selector = (
        TackleSelector(
            profile,
            keeper_cover=mode != "selector",
            keeper_timing=mode == "keeper-timing",
            keeper_lane=mode == "keeper-lane",
        )
        if mode in ("selector", "keeper-cover", "keeper-timing", "keeper-lane")
        else None
    )
    keeper = RobotKey(presser.team, 0)
    gap = 10.0
    contact = False
    contact_episodes = 0
    previous_contacts = set()
    goalward = out = own_goal = capture = last_sweep = None
    dwell = 0
    previous = {}
    yaw_target = world.robot(presser).yaw - PIVOT_SIDES.get(mode, 0) * SWEEP_RADIANS
    sign = world.direction(presser.team)
    physics_dt = backend.basic_ms / 1000
    confirmation = round(0.24 / physics_dt)
    cover_times = []
    film = []
    motion = []
    captured_by = dwell_owner = None
    if frames is not None:
        frames.mkdir(parents=True)
        backend.supervisor.getFromDef("CAMERA").getField("position").setSFVec3f(
            [-sign * 4.10, 0, 3.25]
        )

    def record(state):
        name = f"{len(film):04d}.jpg"
        backend.supervisor.exportImage(str(frames / name), 90)
        film.append(
            dict(
                file=name,
                time=state.time,
                ball=(state.ball.x, state.ball.y),
                presser=(state.robot(presser).x, state.robot(presser).y),
                keeper=(state.robot(keeper).x, state.robot(keeper).y),
            )
        )

    for tick in range(round(SECONDS / CFG.dt)):
        if frames is not None and tick % 2 == 0:
            record(world)
        observed = observed_velocity(world.robots, previous, CFG.dt)
        previous = {r.key: (r.x, r.y) for r in world.robots}
        player, rival = world.robot(presser), world.robot(carrier)
        has_ball = possession(world, presser, CFG)
        rival_holds = possession(world, carrier, CFG)
        goalie = world.robot(keeper)
        keeper_holds = keeper_live and possession(world, keeper, CFG)
        action = mode
        goal_veto = False
        if mode == CONTAIN:
            request = contain_command(world, player)
        elif keeper_holds and not rival_holds:
            request = Command()
        elif has_ball and not rival_holds:
            # All release choices use the same stationary hold for the follow-through.
            request = Command(dribbler=1)
        elif selector:
            fetch = selector.collect_loose(world, player)
            request = fetch or selector.command(world, player, rival, observed, has_ball)
            action = "collect" if fetch else selector.action(presser)
            goal_veto = fetch is None and selector.goal_veto(presser)
        elif mode in PIVOT_SIDES and world.time < SWEEP_SECONDS and has_ball and rival_holds:
            last_sweep = world.time
            request = sweep_command(world, player, rival, PIVOT_SIDES[mode], yaw_target)
        else:
            fetch = fetch_loose(world, player, last_sweep)
            request = fetch or profile.command(world, player, rival, observed, has_ball)
        players = {r.key: Command() for r in world.robots if r.key.team == presser.team}
        players[presser] = request
        if keeper_live:
            tx, ty = keeper_target(world, goalie)
            angle = math.atan2(world.ball.y - goalie.y, world.ball.x - goalie.x)
            players[keeper] = (
                Command(dribbler=1)
                if keeper_holds and not rival_holds
                else move(goalie, limit(tx, 4.32), limit(ty, 2.76), angle, speed=1.75)
            )
        cover = {}
        handoffs = {}
        pair = {}
        challenges = {presser: rival} if action != CONTAIN and rival_holds else {}
        if selector and selector.cover:
            pair = selector.cover.commands(world, player, action == CONTAIN, preparing=goal_veto)
            if pair:
                players.update(pair)
                cover[keeper] = -sign
                if mode in ("keeper-timing", "keeper-lane"):
                    handoffs[keeper] = presser
                if presser in pair:
                    cover_times.append(world.time)
                    challenges.pop(presser, None)
                if rival_holds:
                    challenges[keeper] = rival
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
            challenges=challenges,
            goal_cover=cover,
            keeper_handoffs=handoffs,
        )
        commands[carrier] = hold_carrier(seed, world.time)
        if motion_audit:
            ball = world.ball
            motion.append(
                dict(
                    time=world.time,
                    ball=(ball.x, ball.y, ball.vx, ball.vy),
                    action=action,
                    cover=bool(cover),
                    yielding=presser in pair,
                    robots=[
                        [
                            asdict(r.key),
                            r.x,
                            r.y,
                            r.yaw,
                            r.vx,
                            r.vy,
                            possession(world, r.key, CFG),
                            asdict(commands[r.key]),
                        ]
                        for r in (player, goalie, rival)
                    ],
                )
            )
        world = backend.step(commands)
        for state in backend.last_substep_worlds:
            b = state.ball
            participants = (presser, keeper) if keeper_live or cover_times else (presser,)
            touching = set()
            for key in participants:
                r = state.robot(key)
                for q in state.robots:
                    if q.key == key:
                        continue
                    clearance = math.hypot(r.x - q.x, r.y - q.y) - 2 * CFG.robot_radius
                    gap = min(gap, clearance)
                    if clearance <= 0.001:
                        touching.add(frozenset((key, q.key)))
            contact |= bool(touching)
            contact_episodes += len(touching - previous_contacts)
            previous_contacts = touching
            holders = [
                key
                for key in participants
                if commands[key].dribbler > 0.5 and possession(state, key, CFG)
            ]
            held = possession(state, carrier, CFG)
            receiver = holders[0] if len(holders) == 1 and not held else None
            if receiver != dwell_owner:
                dwell_owner, dwell = receiver, 0
            dwell = dwell + 1 if receiver else 0
            if dwell >= confirmation and capture is None:
                capture = state.time
                captured_by = "keeper" if receiver == keeper else "presser"
            if not holders and not held and goalward is None and toward_own_goal(b, sign):
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
    if frames is not None:
        record(world)
    return dict(
        initial=initial,
        capture_s=capture,
        captured_by=captured_by,
        cover_ticks=len(cover_times),
        first_cover_s=cover_times[0] if cover_times else None,
        keeper_ready_ticks=len(selector.cover.prepared) if selector and selector.cover else 0,
        film=film,
        motion=motion,
        retained=dwell >= confirmation,
        clean_retained=dwell >= confirmation
        and not contact
        and out is None
        and own_goal is None
        and (goalward is None or keeper_live),
        body_contact=contact,
        body_contact_episodes=contact_episodes,
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
    keepers = zone in ("keeper-cover", "keeper-timing", "keeper-lane")
    modes = (
        ("keeper-cover", zone)
        if zone in ("keeper-timing", "keeper-lane")
        else ("selector", "keeper-cover")
        if keepers
        else PANEL_ACTIONS
    )
    keeper_live = keepers and not cfg.get("parked_keeper")
    output = []
    for seed in range(cfg["seed"], cfg["seed"] + cfg["tackle_trials"]):
        start, carrier, presser = opening("own-goal" if keepers else zone, seed)
        if keeper_live:
            sign = start.direction(presser.team)
            setup = replace(
                start.robot(RobotKey(presser.team, 0)),
                x=-sign * 4.24,
                y=-math.copysign(0.29, start.ball.y),
                yaw=0.0 if sign > 0 else math.pi,
            )
            start = replace(
                start, robots=tuple(setup if r.key == setup.key else r for r in start.robots)
            )
            if cfg.get("keeper_stress") and (seed // 4) % 2 == 0:
                # Half the starts carry additional goalward momentum with the
                # ball already shared close to the line.
                start = replace(start, ball=replace(start.ball, vx=start.ball.vx - sign * 0.10))
        outcomes = {
            mode: trial(
                backend,
                policy,
                profile,
                start,
                carrier,
                presser,
                seed,
                zone,
                mode,
                keeper_live=keeper_live,
                frames=folder / "frames" / mode if cfg.get("frame_images") else None,
                motion_audit=cfg.get("motion_audit", False),
            )
            for mode in modes
        }
        initial = next(iter(outcomes.values()))["initial"]
        if any(r.pop("initial") != initial for r in outcomes.values()):
            raise RuntimeError("Matched choices started from different native states")
        output.append(dict(zone=zone, seed=seed, initial=initial, outcomes=outcomes))
    (folder / "tackle-panel.json").write_text(json.dumps(output, indent=2) + "\n")
