"""Fit and compare solo first-touch profiles on matched native Webots serves."""

from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import random
from statistics import median
import traceback

from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.five_receive import FiveReceive
from roboturk.five_roles import CFG
from roboturk.model import Ball, Robot, RobotKey, World, possession, wrap


KEY = RobotKey("blue", 1)
SPEED_BANDS = ((1.4, 2.0), (2.0, 3.0), (3.0, 4.4))
HOLD_S = 0.25
RELATIVE_SPEED = 0.35
TRAIN_COUNT = 15


def serve(seed):
    rng = random.Random(seed)
    angle = rng.uniform(-math.pi, math.pi)
    dx, dy = math.cos(angle), math.sin(angle)
    speed = rng.uniform(*SPEED_BANDS[seed % len(SPEED_BANDS)])
    distance = rng.uniform(0.65, 1.25)
    offset = rng.uniform(-0.21, 0.21)
    heading = wrap(angle + math.pi + rng.uniform(-0.9, 0.9))
    robot = Robot(
        KEY, CFG.possession_x * dx - offset * dy, CFG.possession_x * dy + offset * dx, heading
    )
    ball = Ball(-distance * dx, -distance * dy, speed * dx, speed * dy)
    return World(0, (robot,), ball)


def trial(backend, seed, profile, film=None):
    world = backend.reset(serve(seed))
    start = asdict(world)
    start["robots"] = list(start["robots"])
    first_touch = rebound = secured = None
    hold_start = None
    if film:
        folder, title = film
        folder.mkdir(parents=True, exist_ok=True)
    for tick in range(round(3.0 / CFG.dt)):
        if film:
            state = (
                "Secure"
                if secured is not None
                else "Lost first touch"
                if rebound is not None
                else "Receiving"
            )
            label = f"{title}  |  {world.time:.2f} s  |  {state}"
            backend.supervisor.setLabel(0, label, 0.04, 0.04, 0.075, 0xFFFFFF, 0, "Arial")
            backend.supervisor.exportImage(str(folder / f"{tick:03d}.jpg"), 85)
        actor = world.robot(KEY)
        command = profile.command(world, actor, (0.0, 0.0))
        world = backend.step({KEY: command})
        for sample in backend.last_substep_worlds:
            robot, ball = sample.robot(KEY), sample.ball
            x, y = ball.x - robot.x, ball.y - robot.y
            distance = math.hypot(x, y)
            vx, vy = ball.vx - robot.vx, ball.vy - robot.vy
            in_mouth = possession(sample, KEY, CFG)
            if first_touch is None and (in_mouth or distance <= CFG.robot_radius + CFG.ball_radius):
                first_touch = sample.time
            if secured is not None or rebound is not None:
                continue
            if first_touch is not None and distance > 0.175 and x * vx + y * vy > 0:
                rebound = sample.time
            elif in_mouth and math.hypot(vx, vy) <= RELATIVE_SPEED:
                if hold_start is None:
                    hold_start = sample.time
                if sample.time - hold_start >= HOLD_S - 1e-9:
                    secured = sample.time
            else:
                hold_start = None
        if film is None and (secured is not None or rebound is not None):
            break
    return dict(
        seed=seed, start=start, first_touch_s=first_touch, rebound_s=rebound, secured_s=secured
    )


def evaluate(backend, seeds, profiles):
    outcomes = {name: [] for name in profiles}
    for index, seed in enumerate(seeds):
        names = list(profiles) if index % 2 == 0 else reversed(profiles)
        for name in names:
            outcomes[name].append(trial(backend, seed, profiles[name]))
        starts = [cases[-1]["start"] for cases in outcomes.values()]
        if any(start != starts[0] for start in starts):
            raise RuntimeError(f"Native resets differ on matched serve {seed}")
    return outcomes


def metrics(rows):
    secured = [r for r in rows if r["secured_s"] is not None]
    return dict(
        clean_catches=len(secured),
        first_touch_losses=sum(r["rebound_s"] is not None for r in rows),
        no_touch=sum(r["first_touch_s"] is None for r in rows),
        median_touch_to_hold_s=median(r["secured_s"] - r["first_touch_s"] for r in secured)
        if secured
        else None,
    )


def run(supervisor, mode):
    dest = Path(os.environ["ROBOTURK_OUTPUT"])
    dest.parent.mkdir(parents=True, exist_ok=True)
    backend = ContinuousWebotsBackend(supervisor, CFG)
    try:
        if mode == "receive-dojo-demo":
            report = json.loads(Path(os.environ["ROBOTURK_CHECKPOINT"]).read_text())
            film_comparison(backend, report, dest)
        else:
            train_and_evaluate(backend, dest)
    except Exception:
        traceback.print_exc()
        dest.with_suffix(".error.txt").write_text(traceback.format_exc())
        supervisor.simulationQuit(1)
    else:
        supervisor.simulationQuit(0)


def train_and_evaluate(backend, dest):
    root = Path(__file__).resolve().parents[5]
    published = json.loads((root / "policies/five_receive_v1/policy.json").read_text())
    previous = FiveReceive(**published)
    profiles = {"released": previous}
    profiles["gain-only"] = FiveReceive(1.0, 3.0, previous.opening)
    for cushion_range in (0.85, 1.2, 1.5):
        for cushion_speed in (1.8, 2.5):
            profiles[f"{cushion_range:g} m / {cushion_speed:g} m/s"] = FiveReceive(
                1.0, 3.0, previous.opening, cushion_range, cushion_speed
            )
    seed = int(os.environ["ROBOTURK_SEED"])
    episodes = int(os.environ["ROBOTURK_EPISODES"])
    tuning = evaluate(backend, range(seed, seed + TRAIN_COUNT), profiles)
    scores = {name: metrics(rows) for name, rows in tuning.items()}

    def rank(name):
        score = scores[name]
        latency = score["median_touch_to_hold_s"]
        return score["clean_catches"], -score["first_touch_losses"], -(latency or math.inf)

    winner = max(profiles, key=rank)
    pair = {"released": previous, "fitted": profiles[winner]}
    held_out_seed = seed + 1000
    held_out = evaluate(backend, range(held_out_seed, held_out_seed + episodes), pair)
    summary = {name: metrics(rows) for name, rows in held_out.items()}
    report = dict(
        task="solo ground reception",
        backend=backend.name,
        control_step_s=CFG.dt,
        physics_step_ms=backend.basic_ms,
        hold_s=HOLD_S,
        hold_relative_speed_m_s=RELATIVE_SPEED,
        fit_seed=seed,
        fit_count=TRAIN_COUNT,
        held_out_seed=held_out_seed,
        held_out_count=episodes,
        selected_profile=winner,
        profiles={name: asdict(profile) for name, profile in profiles.items()},
        fitting=scores,
        held_out=summary,
        episodes=held_out,
    )
    dest.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(summary, "profile", winner, flush=True)


def film_comparison(backend, report, dest):
    camera = backend.supervisor.getFromDef("CAMERA")
    camera.getField("position").setSFVec3f([0, 0, 3.4])
    held_out = report["episodes"]
    selected = next(
        (
            i
            for i, (a, b) in enumerate(zip(*held_out.values(), strict=True))
            if a["secured_s"] is None and b["secured_s"] is not None
        ),
        0,
    )
    filmed = {}
    for name, title in (("released", "Current receiver"), ("fitted", "Fitted receiver")):
        profile_name = "released" if name == "released" else report["selected_profile"]
        profile = FiveReceive(**report["profiles"][profile_name])
        original = held_out[name][selected]
        example = trial(backend, original["seed"], profile, (dest.parent / name, title))
        for field in ("start", "first_touch_s", "rebound_s", "secured_s"):
            a, b = example[field], original[field]
            matched = (
                math.isclose(a, b, abs_tol=backend.basic_ms / 1000 + 1e-8)
                if isinstance(a, float) and isinstance(b, float)
                else a == b
            )
            if not matched:
                raise RuntimeError(f"Filmed trial did not reproduce {name} {field}")
        filmed[name] = example
    dest.write_text(json.dumps(filmed, indent=2, allow_nan=False) + "\n")
