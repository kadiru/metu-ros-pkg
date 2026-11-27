"""Measure full stops on paired fresh Webots starts, including incoming velocity."""

from dataclasses import replace
import json
import os
from pathlib import Path
from statistics import median
import traceback

from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.model import unit
from roboturk.primitive_dojo import ACTOR, CFG, scenario

from primitive_dojo_runner import PROFILES, Trial, arrival_error


def start(seed, moving):
    drill = scenario("arrival", seed * 2)
    if not moving:
        return drill
    robot = drill.world.robot(ACTOR)
    goal = drill.arrival.point
    dx, dy = unit(goal[0] - robot.x, goal[1] - robot.y)
    speed = 1.8 + 0.11 * (seed % 6)
    # Just beyond the measured state's theoretical braking distance. Keep a
    # small cross-track velocity so the controller must settle both axes.
    distance = speed**2 / (2 * CFG.max_acceleration) + 0.05 + (seed % 4) * 0.02
    lateral = ((seed % 5) - 2) * 0.15
    robot = replace(
        robot,
        x=goal[0] - distance * dx,
        y=goal[1] - distance * dy,
        vx=speed * dx - lateral * dy,
        vy=speed * dy + lateral * dx,
    )
    return replace(drill, world=replace(drill.world, robots=(robot,)))


def trial(backend, drill, profile):
    controller = Trial(drill, profile, {})
    world = backend.reset(drill.world)
    origin = world.robot(ACTOR)
    tx, ty = drill.arrival.point
    ux, uy = unit(tx - origin.x, ty - origin.y)
    settled = None
    earliest = None
    overshoot = 0.0
    final_error = None
    for _ in range(round(3.2 / CFG.dt)):
        world = backend.step(controller.commands(world))
        for frame in backend.last_substep_worlds:
            robot = frame.robot(ACTOR)
            overshoot = max(overshoot, (robot.x - tx) * ux + (robot.y - ty) * uy)
            final_error = arrival_error(robot, drill.arrival)
            distance, angle, speed = final_error
            if distance <= 0.01 and speed <= 0.05 and angle <= 0.04:
                earliest = frame.time if earliest is None else earliest
                if frame.time - earliest >= 0.25 and settled is None:
                    settled = earliest
            else:
                earliest = None
    return dict(settled_s=settled, overshoot_m=overshoot, final_error=final_error)


def summarize(rows):
    times = [r["settled_s"] for r in rows if r["settled_s"] is not None]
    return dict(
        trials=len(rows),
        settled=len(times),
        median_settled_s=median(times) if times else None,
        crossed_by_1cm=sum(r["overshoot_m"] > 0.01 for r in rows),
        worst_overshoot_m=max(r["overshoot_m"] for r in rows),
    )


def run(supervisor):
    try:
        backend = ContinuousWebotsBackend(supervisor, CFG)
        seed = int(os.environ["ROBOTURK_SEED"])
        count = int(os.environ["ROBOTURK_EPISODES"])
        results = {}
        for moving in (False, True):
            rows = {"proportional": [], "braking": []}
            for case in range(seed, seed + count):
                drill = start(case, moving)
                profiles = PROFILES["arrival"]
                for name, profile in (
                    ("proportional", profiles["fast"]),
                    ("braking", profiles["braking"]),
                ):
                    rows[name].append({"seed": case, **trial(backend, drill, profile)})
            results["moving_start" if moving else "rest_start"] = {
                name: {"summary": summarize(arm), "episodes": arm} for name, arm in rows.items()
            }
        path = Path(os.environ["ROBOTURK_OUTPUT"])
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(results, indent=2, allow_nan=False) + "\n")
        print(
            {
                panel: {name: d["summary"] for name, d in arms.items()}
                for panel, arms in results.items()
            },
            flush=True,
        )
    except Exception:
        traceback.print_exc()
        supervisor.simulationQuit(1)
    else:
        supervisor.simulationQuit(0)
