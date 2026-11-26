"""Frozen-policy transfer to a flat-chest collision model and shaft envelope."""

from dataclasses import asdict, replace
import json
import math
import os

from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.model import Ball, Command, Referee, Robot, World, unit
from roboturk.primitive_dojo import ACTOR, CFG, scenario
from roboturk.wheel_drive import WHEEL_ANGLES_DEG, beam_broken, limit_by_shafts

from primitive_dojo_runner import stats, trial


MAX_SHAFT_RAD_S = 90.0


class FlatChestBackend(ContinuousWebotsBackend):
    name = "webots_flat_chest_shaft_envelope"
    robot_proto = "SslFlatChestRobot"

    def step(self, commands):
        return super().step(
            {key: limit_by_shafts(command, MAX_SHAFT_RAD_S) for key, command in commands.items()}
        )


def actuator_smoke(backend, elevation):
    start = World(0, (Robot(ACTOR, 0, 0),), Ball(CFG.possession_x, 0), Referee.RUNNING)
    world = backend.reset(start)
    before = beam_broken(world.robot(ACTOR), world.ball, CFG.ball_radius)
    world = backend.step({ACTOR: Command(dribbler=1)})
    mouth = beam_broken(world.robot(ACTOR), world.ball, CFG.ball_radius)
    world = backend.step({ACTOR: Command(kick=3.2, kick_elevation=elevation)})
    if len(backend.last_kick_events) != 1:
        raise RuntimeError("Actuator smoke test did not apply exactly one kick")
    _, release, launch = backend.last_kick_events[0]
    apex = max(s.ball.z for s in backend.last_substep_worlds)
    for _ in range(45):
        world = backend.step({})
        apex = max(apex, *(s.ball.z for s in backend.last_substep_worlds))
    return dict(
        loft_rad=elevation,
        launch_m_s=launch,
        release=asdict(release.ball),
        beam_before=before,
        beam_after_roller=mouth,
        beam_after=beam_broken(world.robot(ACTOR), world.ball, CFG.ball_radius),
        peak_height_m=apex,
        final_ball=asdict(world.ball),
    )


def off_axis_arrivals(backend, model, seed, count):
    profile = model["profiles"][model["selected"]["moving"]]
    arms = {"direct": [], "with_runup": []}
    for index in range(count):
        start = seed + 2 * index + 1
        drill = scenario("arrival", start)
        robot = drill.world.robot(ACTOR)
        ux, uy = unit(*drill.arrival.velocity)
        lateral = 0.85 if index % 2 else -0.85
        along = 0.95 if index % 4 < 2 else 0.10
        x, y = drill.arrival.point
        robot = replace(robot, x=x + along * ux - lateral * uy, y=y + along * uy + lateral * ux)
        drill = replace(drill, context="off axis", world=replace(drill.world, robots=(robot,)))
        for name, rows in arms.items():
            rows.append(
                trial(
                    backend,
                    "arrival",
                    start,
                    name,
                    {**profile, "route": name == "with_runup"},
                    {},
                    drill=drill,
                )
            )
    return {name: dict(summary=stats(rows), episodes=rows) for name, rows in arms.items()}


def audit(ideal, dest, tasks, checkpoint):
    seed = int(os.environ["ROBOTURK_SEED"])
    count = int(os.environ["ROBOTURK_EPISODES"])
    eligible = range(seed, seed + count)
    for task in tasks:
        train_seed = checkpoint["tasks"][task].get("train_seed", checkpoint["train_seed"])
        if seed < train_seed + 3000 and seed + count > train_seed:
            raise ValueError(
                f"Use transfer seeds outside fitting, probe and qualification for {task}"
            )

    def evaluate(backend):
        outcomes = {}
        for task in tasks:
            model = checkpoint["tasks"][task]
            rows = []
            for start in eligible:
                name = model["selected"][scenario(task, start).context]
                rows.append(
                    trial(
                        backend,
                        task,
                        start,
                        name,
                        model["profiles"][name],
                        model["fixed_race_passer"],
                    )
                )
            outcomes[task] = dict(summary=stats(rows), episodes=rows)
        return outcomes

    ideal_result = evaluate(ideal)
    off_axis = (
        off_axis_arrivals(ideal, checkpoint["tasks"]["arrival"], seed + 100, count)
        if "arrival" in tasks
        else None
    )
    for node in ideal.nodes.values():
        node.remove()
    ideal.nodes.clear()
    if ideal.supervisor.step(ideal.basic_ms) == -1:
        raise RuntimeError("Webots ended while switching to the flat chest")
    flat = FlatChestBackend(ideal.supervisor, CFG)
    flat_result = evaluate(flat)
    smoke = {
        name: actuator_smoke(flat, angle)
        for name, angle in (("flat", 0), ("chip", math.radians(36)))
    }
    if not all(
        r["beam_before"] and r["beam_after_roller"] and not r["beam_after"] for r in smoke.values()
    ):
        raise RuntimeError("The flat-chest beam proxy did not clear after release")
    if smoke["chip"]["peak_height_m"] < 0.12 or smoke["flat"]["peak_height_m"] > 0.04:
        raise RuntimeError("Flat and lofted trajectories did not separate in the smoke test")
    report = dict(
        wheel_angles_deg=WHEEL_ANGLES_DEG,
        wheel_limit_rad_s=MAX_SHAFT_RAD_S,
        start_seed=seed,
        count=count,
        original=ideal_result,
        flat_chest=flat_result,
        off_axis_arrivals=off_axis,
        functional_checks=smoke,
    )
    dest.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(
        {
            a: {t: d["summary"]["successes"] for t, d in out.items()}
            for a, out in (("original", ideal_result), ("flat_chest", flat_result))
        },
        flush=True,
    )
