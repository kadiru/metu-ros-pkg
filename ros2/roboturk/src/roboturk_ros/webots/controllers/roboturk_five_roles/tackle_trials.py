"""Paired 5v5-field Presser challenges against a scripted, roller-held carrier."""

from dataclasses import asdict, replace
import gzip
import json
import math
import random

from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.five_play import commands
from roboturk.five_roles import Assignment, CFG, kickoff, TEAMS
from roboturk.model import Ball, Command, RobotKey, possession
from roboturk.pivot_fetch import PivotFetch
from roboturk.open_motion import Steering, observed_velocity, steer_team
from roboturk.tackle import BallSideTackle
from roboturk.three_match import snapshot

SECONDS = 4.4


def opening(seed, duel=False):
    rng = random.Random(seed)
    team = TEAMS[(seed // 2) % 2]
    carrier = RobotKey(team, 1)
    presser = RobotKey(TEAMS[1 - TEAMS.index(team)], 1)
    yaw = rng.uniform(-0.24, 0.24) + (0 if seed % 2 == 0 else math.pi)
    x, y = rng.uniform(-0.55, 0.55), rng.uniform(-0.4, 0.4)
    direction = yaw + math.radians(rng.uniform(9, 72)) * rng.choice((-1, 1))
    gap = rng.uniform(0.46, 0.77)
    if duel:
        gap = rng.uniform(0.277, 0.318)
        direction = yaw + math.radians(rng.uniform(8, 42)) * rng.choice((-1, 1))
    px, py = x + gap * math.cos(direction), y + gap * math.sin(direction)
    ball = Ball(x + CFG.possession_x * math.cos(yaw), y + CFG.possession_x * math.sin(yaw))
    w = kickoff(seed, team, 1 if seed % 2 == 0 else -1)
    positions = {carrier: (x, y, yaw), presser: (px, py, math.atan2(ball.y - py, ball.x - px))}
    parked = {0: (4.18, 0), 2: (3.35, -1.96), 3: (2.65, 1.95), 4: (1.85, -1.72)}
    robots = []
    for r in w.robots:
        key = r.key
        if key in positions:
            rx, ry, turn = positions[key]
        else:
            rx, ry = parked[key.id]
            sign = -1 if key.team == "blue" else 1
            rx, ry, turn = rx * sign, ry * sign, r.yaw
        robots.append(replace(r, x=rx, y=ry, yaw=turn))
    return replace(w, robots=tuple(robots), ball=ball), carrier, presser


def carrier_command(seed, t, duel=False):
    kind = seed % 4
    speed = (0, 0.16, 0.25, 0.17)[kind]
    direction = 1 if seed // 4 % 2 else -1
    if duel:
        speed = (0, -0.17, 0.19, 0.20)[kind]
        turn = direction * (0.66 if kind == 3 else 0)
        slide = direction * 0.19 * math.sin(2.2 * t) if kind == 2 else 0
        return Command(vx=speed, vy=slide, omega=turn, dribbler=1)
    turn = direction * (0.27 if kind == 3 else 0)
    return Command(
        vx=speed,
        vy=direction * 0.09 * math.sin(1.4 * t) if kind == 2 else 0,
        omega=turn,
        dribbler=1,
    )


def trial(sim, backend, policy, cfg, seed, tackle, frames):
    duel = cfg.get("tackle_duel", False)
    opening_world, carrier, presser = opening(seed, duel)
    if tackle and cfg.get("pivot_fetch"):
        tackle = PivotFetch(tackle)
    w = backend.reset(opening_world)
    initial = asdict(w)
    assert possession(w, carrier, CFG) and not possession(w, presser, CFG)
    if frames:
        ball = w.ball
        sim.getFromDef("CAMERA").getField("position").setSFVec3f([ball.x, ball.y, 3.7])
    min_gap = 10.0
    contacts = set()
    mouth = capture = first_contact = None
    dwell = 0
    previous = {}
    samples = [] if cfg.get("motion_audit") or frames else None
    control = []
    dt = CFG.dt
    physics_dt = backend.basic_ms / 1000
    for tick in range(round(SECONDS / dt)):
        if frames and tick % 2 == 0:
            sim.exportImage(str(frames / f"{tick // 2:04d}.jpg"), 89)
        measured = observed_velocity(w.robots, previous, dt)
        previous = {r.key: (r.x, r.y) for r in w.robots}
        roles = {
            r.key: Assignment("Presser" if r.key == presser else "Keeper", (r.x, r.y))
            for r in w.robots
        }
        has_ball = possession(w, presser, CFG)
        rival = w.robot(carrier)
        player = w.robot(presser)
        if tackle:
            fetch = tackle.collect_loose(w, player) if isinstance(tackle, PivotFetch) else None
            request = fetch or tackle.command(w, player, rival, measured, has_ball)
        else:
            request = commands(w, roles, None, None, None)[0][presser]
        team_commands = {
            r.key: request if r.key == presser else Command()
            for r in w.robots
            if r.key.team == presser.team
        }
        out = steer_team(
            w.robots,
            presser.team,
            team_commands,
            measured,
            policy,
            dt,
            challenges={presser: rival} if tackle else None,
        )
        out[carrier] = carrier_command(seed, w.time, duel)
        if samples is not None:
            samples.append(snapshot(w) + [backend.ball.getPosition()[2]])
            control.append([possession(w, carrier, CFG), has_ball])
        w = backend.step(out)
        for state in backend.last_substep_worlds:
            a, b = state.robot(carrier), state.robot(presser)
            for robot in state.robots:
                if robot.key == presser:
                    continue
                gap = math.hypot(robot.x - b.x, robot.y - b.y) - 2 * CFG.robot_radius
                min_gap = min(min_gap, gap)
                if gap <= 0.001:
                    contacts.add(robot.key)
                    if first_contact is None:
                        first_contact = state.time
            received = possession(state, presser, CFG)
            if received and mouth is None:
                mouth = state.time
            exclusive = received and not possession(state, a.key, CFG)
            dwell = dwell + 1 if exclusive else 0
            if dwell >= round(0.24 / physics_dt) and capture is None:
                capture = state.time
    clean = capture is not None and not contacts
    return dict(
        seed=seed,
        carrier=asdict(carrier),
        presser=asdict(presser),
        initial=initial,
        clean_takeaway=clean,
        capture_s=capture,
        roller_s=mouth,
        min_gap_m=min_gap,
        contact=bool(contacts),
        first_contact_s=first_contact,
        sweep_times=[t for t, _, _ in tackle.sweeps] if isinstance(tackle, PivotFetch) else [],
        trace=samples,
        control=control or None,
    )


def run(sim, folder, cfg):
    policy = Steering(**cfg["open_play_policy"])
    backend = ContinuousWebotsBackend(sim, CFG)
    sweep = cfg.get("tackle_sweep") or {
        "tackle" if cfg.get("tackle_policy") else "current": cfg.get("tackle_policy")
    }
    for name, weights in sweep.items():
        tackle = BallSideTackle(**weights) if weights else None
        dest = folder / name
        dest.mkdir()
        frames = dest / "frames" if cfg["frame_images"] else None
        if frames:
            frames.mkdir()
        with gzip.open(dest / "trials.jsonl.gz", "wt", compresslevel=3) as stream:
            for seed in range(cfg["seed"], cfg["seed"] + cfg["tackle_trials"]):
                row = trial(sim, backend, policy, cfg, seed, tackle, frames)
                stream.write(json.dumps(row, separators=(",", ":")) + "\n")
                stream.flush()
