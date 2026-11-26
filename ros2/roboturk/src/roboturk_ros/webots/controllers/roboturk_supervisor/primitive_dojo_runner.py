"""Collect and fit motor profiles in native Webots, with disjoint seed panels."""

from dataclasses import asdict, replace
import gzip
import json
import math
import os
from pathlib import Path
from statistics import mean, median
import tempfile
import traceback

from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.five_receive import carrying
from roboturk.model import Command, body_vector, possession, wrap
from roboturk.primitive_dojo import (
    ACTOR,
    CFG,
    DEFENDER,
    RECEIVE,
    RECEIVER,
    SERVERS,
    TASKS,
    Arrival,
    ArrivalRoute,
    deflection_station,
    go_to,
    intercept,
    pass_command,
    pass_plan,
    scenario,
    serve_relay,
    xy,
)

PROFILES = {
    "arrival": {
        "baseline": {"terminal": False},
        "terminal": {},
        "soft": {"gain": 2.0, "speed": 1.85},
        "fast": {"gain": 3.2, "speed": 2.4},
        "precise": {"gain": 4.8, "speed": 1.9},
        "gentle": {"gain": 2.5, "speed": 1.4},
    },
    "intercept": {
        "baseline": {},
        "early": {"predict": True, "speed": 1.6},
        "quick": {"predict": True, "speed": 2.2, "margin": 0.1},
        "cautious": {"predict": True, "speed": 1.5, "margin": 0.16},
        "close": {"predict": True, "speed": 1.9, "margin": 0, "gain": 6.0},
        "late": {"predict": True, "speed": 2.2, "margin": -0.10},
        "station": {"station": {"speed": 1.65, "settle": 0.32, "restitution": 0.20}},
        "station_soft": {"station": {"speed": 1.65, "settle": 0.37, "restitution": 0.02}},
        "station_bounce": {"station": {"speed": 1.65, "settle": 0.32, "restitution": 0.45}},
        "station_wide": {
            "station": {"speed": 1.85, "settle": 0.38, "restitution": 0.20, "normal_bias": 8}
        },
    },
    "pass": {
        "baseline": {},
        "soft": {"lead": 0.82, "base": 1.68, "power": 0.21},
        "firm": {"lead": 0.65, "base": 2.22, "power": 0.28},
        "full_lead": {"lead": 1.08, "base": 1.9, "power": 0.24},
        "short_lead": {"lead": 0.4, "base": 1.76, "power": 0.25},
        "feet": {"base": 1.42, "power": 0.22},
    },
    "receive_race": {
        "baseline": {"wait": True},
        "quick": {"predict": True, "speed": 2.2, "margin": 0.02},
        "early": {"predict": True, "speed": 1.7, "margin": -0.06},
        "premeet_short": {"predict": True, "speed": 1.7, "margin": -0.06, "premeet": 0.32},
        "premeet_long": {"predict": True, "speed": 1.7, "margin": -0.06, "premeet": 0.53},
        "reactive": {"speed": 2.2},
        "cautious": {"predict": True, "speed": 1.7, "margin": 0.15},
        "delayed": {"predict": True, "speed": 2.2, "delay": 0.24},
        "cutoff": {
            "predict": True,
            "speed": 2.45,
            "margin": -0.08,
            "arrival_fraction": 0.0,
            "cushion_range": 0.65,
        },
        "close_early": {
            "predict": True,
            "speed": 2.25,
            "margin": -0.08,
            "arrival_fraction": 0.0,
            "cushion_range": 0.36,
            "receive_fraction": 0.94,
        },
    },
}
HOLD = 0.20
RELATIVE_SPEED = 0.35
HORIZON = {"arrival": 3.2, "intercept": 6.4, "pass": 3.5, "receive_race": 3.5}
CONTACT = CFG.robot_radius + CFG.ball_radius + 0.003


def vector(entity):
    return entity.vx, entity.vy


def capture(world, key):
    return (
        possession(world, key, CFG)
        and all(robot.key == key or not possession(world, robot.key, CFG) for robot in world.robots)
        and math.dist(vector(world.robot(key)), vector(world.ball)) <= RELATIVE_SPEED
    )


def arrival_error(robot, goal):
    return [
        math.dist(xy(robot), goal.point),
        abs(wrap(robot.yaw - goal.yaw)),
        math.dist(vector(robot), goal.velocity),
    ]


def touches(world, key):
    return possession(world, key, CFG) or math.dist(xy(world.robot(key)), xy(world.ball)) <= CONTACT


def point_text(point):
    return f"{point[0]:+.2f}, {point[1]:+.2f}"


class Trial:
    def __init__(self, drill, profile, passer):
        self.drill = drill
        self.profile = {k: v for k, v in profile.items() if k != "route"}
        self.arrival_route = (
            ArrivalRoute.from_pose(drill.world.robot(ACTOR), drill.arrival)
            if drill.arrival and profile.get("route")
            else None
        )
        self.passer = passer
        self.target = None
        self.pass_power = None
        self.hold_start = None
        self.defender_hold = None
        self.carry_start = None
        self.carry_yaw = None
        self.best_error = None
        self.anchor_speed = None
        self.contact = None
        self.release = None
        self.serve_ready = SERVERS[0]
        self.serve_time = None
        self.station = None
        self.events = []
        self.success = None
        self.failure = None
        self.receiver_touched = None
        self.rival_touched = None
        self.body_contact = None
        self.meeting = None
        self.release_receiver = None
        self.run_progress = None
        self.terminal = None
        self.kicks = 0
        self.best_alignment = None
        self.observed_clear = 0

    def commands(self, world):
        task = self.drill.task
        r = world.robot(ACTOR)
        ball = world.ball
        if task == "arrival":
            if self.arrival_route is not None:
                return {ACTOR: self.arrival_route.command(r, **self.profile)}
            return {ACTOR: go_to(r, self.drill.arrival, **self.profile)}
        if task == "intercept":
            return self.intercept_commands(world, r)
        mate = world.robot(RECEIVER)
        if self.release is None:
            vx, vy = body_vector(0, self.drill.run_speed, mate.yaw)
            if world.time < 0.078:
                return {ACTOR: Command(dribbler=1), RECEIVER: Command(vx, vy)}
            if self.target is None:
                profile = self.profile if task == "pass" else self.passer
                self.target, self.pass_power = pass_plan(ball, mate, profile)
            self.meeting = math.dist(xy(mate), self.target)
            receive = Command(vx, vy)
            if task == "receive_race" and self.profile.get("premeet"):
                dx, dy = self.target[0] - ball.x, self.target[1] - ball.y
                distance = max(0.001, math.hypot(dx, dy))
                advance = self.profile["premeet"] / distance
                waypoint = self.target[0] - advance * dx, self.target[1] - advance * dy
                receive = self.wait_at(mate, ball, waypoint)
            return {
                ACTOR: pass_command(world, r, self.target, self.pass_power),
                RECEIVER: receive,
            }
        if task == "pass":
            # The runner commits to the planned rendezvous. No chasing a missed
            # pass across the field during evaluation.
            if math.dist(xy(ball), self.target) > 0.63 and not possession(world, RECEIVER, CFG):
                receive = self.wait_at(mate, ball, self.target)
            else:
                receive = RECEIVE.command(world, mate, self.target)
            return {RECEIVER: receive}
        if self.profile.get("wait") or world.time - self.release < self.profile.get("delay", 0):
            if math.dist(xy(ball), self.target) > 0.43 and not possession(world, RECEIVER, CFG):
                receive = self.wait_at(mate, ball, self.target)
            else:
                receive = RECEIVE.command(world, mate, self.target)
        else:
            receive = intercept(world, mate, self.profile)
        rival = world.robot(DEFENDER)
        rival_command = intercept(world, rival, {"speed": self.drill.rival_speed, "predict": True})
        return {RECEIVER: receive, DEFENDER: rival_command}

    @staticmethod
    def wait_at(robot, ball, point):
        yaw = math.atan2(ball.y - point[1], ball.x - point[0])
        return go_to(robot, Arrival(point, yaw), speed=1.65)

    def intercept_commands(self, world, r):
        if self.carry_start is not None:
            desired = 0.46 * math.cos(self.carry_yaw), 0.46 * math.sin(self.carry_yaw)
            return {ACTOR: carrying(world, r, desired, self.carry_yaw)}
        station_profile = self.profile.get("station")
        if station_profile and self.drill.outlet is not None:
            if (
                self.station is None
                and self.serve_time is not None
                and world.time - self.serve_time >= 0.20
            ):
                self.station = deflection_station(world, r, self.drill.outlet, **station_profile)
            actor = (
                go_to(r, self.station, speed=station_profile["speed"], gain=5.0)
                if self.station
                else Command()
            )
        else:
            actor = intercept(world, r, self.profile, self.drill.outlet)
        commands = {ACTOR: actor}
        # For a rebound the servers become passive once the learner makes body
        # contact. For misses, the next server traps and sends the ball back.
        if self.contact is not None:
            return commands
        commands.update(serve_relay(world, self.serve_ready, self.drill.serve_speed))
        return commands

    def on_kick(self, key, world, speed):
        self.kicks += 1
        self.events.append(
            dict(kind="kick", t=world.time, who=asdict(key), speed=speed, point=xy(world.ball))
        )
        if self.drill.task == "intercept":
            self.serve_ready = SERVERS[1] if key == SERVERS[0] else SERVERS[0]
            self.serve_time = world.time
            self.station = None
        else:
            self.release = world.time
            self.release_receiver = xy(world.robot(RECEIVER))

    def observe(self, world, previous):
        if self.drill.task == "arrival":
            self.observe_arrival(world)
            return
        if self.kicks == 0:
            return
        ball = world.ball
        if abs(ball.x) > CFG.ball_exit_x or abs(ball.y) > CFG.ball_exit_y:
            self.failure = "ball_out"
            return
        if self.drill.outlet is not None:
            self.observe_deflection(world, previous)
            return
        key = ACTOR if self.drill.task == "intercept" else RECEIVER
        r = world.robot(key)
        if self.receiver_touched is None and touches(world, key):
            self.receiver_touched = world.time
        if self.drill.task == "receive_race":
            opponent = world.robot(DEFENDER)
            if self.rival_touched is None and touches(world, DEFENDER):
                self.rival_touched = world.time
            if self.body_contact is None and math.dist(xy(r), xy(opponent)) <= 2 * CFG.robot_radius:
                self.body_contact = world.time
            self.defender_hold = self.hold_time(world, DEFENDER, self.defender_hold)
            if self.defender_hold is not None and world.time - self.defender_hold >= HOLD:
                self.failure = "opponent_secured"
                return
        self.hold_start = self.hold_time(world, key, self.hold_start)
        if self.carry_start is not None:
            if not possession(world, key, CFG):
                self.failure = "lost_after_trap"
            elif math.dist(xy(r), self.carry_start) >= 0.22 and self.hold_start is not None:
                self.success = world.time
        elif self.hold_start is not None and world.time - self.hold_start >= HOLD:
            if key == ACTOR:
                self.carry_start = xy(r)
                self.carry_yaw = r.yaw
                self.events.append(dict(kind="trapped", t=world.time, point=xy(ball)))
            else:
                # Receiver must control the pass near its nominated meeting point
                # in the isolated drill. In the race, earlier control is the goal.
                self.observed_clear = math.dist(xy(ball), self.target)
                self.run_progress = (r.y - self.release_receiver[1]) * (
                    1 if self.drill.run_speed > 0 else -1
                )
                if self.drill.task == "pass" and self.observed_clear > 0.46:
                    self.failure = "outside_receiving_zone"
                elif self.drill.context == "on the run" and self.run_progress < 0.42:
                    self.failure = "failed_to_lead_runner"
                else:
                    self.success = world.time

    @staticmethod
    def hold_time(world, key, started):
        if capture(world, key):
            return world.time if started is None else started
        return None

    def observe_arrival(self, world):
        goal = self.drill.arrival
        robot = world.robot(ACTOR)
        error = arrival_error(robot, goal)
        if self.best_error is None or error[0] < self.best_error[0]:
            self.best_error = error
            self.anchor_speed = math.hypot(*vector(robot))
        if error[0] <= 0.09 and error[1] <= 0.12 and error[2] <= 0.24:
            if math.hypot(*goal.velocity) > 0:
                self.success = world.time
            else:
                self.hold_start = world.time if self.hold_start is None else self.hold_start
                if world.time - self.hold_start >= 0.12:
                    self.success = world.time
        else:
            self.hold_start = None

    def observe_deflection(self, world, previous):
        ball = world.ball
        actor = world.robot(ACTOR)
        if self.contact is None and math.dist(xy(actor), xy(ball)) <= CONTACT:
            if math.dist(vector(ball), vector(previous.ball)) > 0.20:
                self.contact = xy(ball)
                self.events.append(dict(kind="body_deflection", t=world.time, point=self.contact))
        if self.contact is not None and math.dist(xy(ball), self.contact) >= 0.28:
            target = self.drill.outlet
            a = math.atan2(ball.y - self.contact[1], ball.x - self.contact[0])
            b = math.atan2(target[1] - self.contact[1], target[0] - self.contact[0])
            self.best_alignment = abs(wrap(a - b))
            if self.best_alignment <= math.radians(28):
                self.success = world.time
            else:
                self.failure = "off_target_deflection"

    def result(self, world):
        task = self.drill.task
        ok = self.success is not None
        reward = 2 * float(ok) - 0.08 * (self.success if ok else world.time)
        if self.best_error is not None:
            x, yaw, speed = self.best_error
            reward -= 0.2 * min(1, x) + 0.1 * min(1, yaw) + 0.3 * min(1.5, speed)
        elif task == "intercept" and self.drill.outlet is not None:
            reward -= 0.25 * min(2, self.best_alignment if self.best_alignment is not None else 2)
        else:
            reward += 0.15 * (self.receiver_touched is not None)
            reward -= 0.45 * (self.failure == "opponent_secured") + 0.25 * (
                self.body_contact is not None
            )
        reward -= 0.25 * (self.failure == "ball_out")
        return dict(
            context=self.drill.context,
            success_s=self.success,
            failure=self.failure or ("timeout" if not ok else None),
            return_value=reward,
            closest_arrival_error=self.best_error,
            closest_speed=self.anchor_speed,
            target_speed=math.hypot(*self.drill.arrival.velocity) if self.drill.arrival else None,
            deflection_error_rad=self.best_alignment,
            first_touch_s=self.receiver_touched,
            defender_touch_s=self.rival_touched,
            body_contact_s=self.body_contact,
            release_s=self.release,
            lead_m=self.meeting,
            receiver_progress_m=self.run_progress,
            receiving_zone_error_m=self.observed_clear if ok and self.release is not None else None,
            kicks=self.kicks,
            events=self.events,
        )


def trial(backend, task, seed, name, profile, passer, log=None, film=None, *, drill=None):
    drill = drill or scenario(task, seed)
    replay = Trial(drill, profile, passer)
    if film:
        frame_camera(backend.supervisor, drill)
    world = backend.reset(drill.world)
    start = asdict(world)
    ticks = []
    horizon = 4.6 if drill.context == "off axis" else HORIZON[task]
    for tick in range(round(horizon / CFG.dt)):
        commands = replay.commands(world)
        if log:
            ticks.append(
                {
                    "world": asdict(world),
                    "commands": {
                        f"{k.team[0].upper()}{k.id}": asdict(c) for k, c in commands.items()
                    },
                }
            )
        if film and tick % 2 == 0:
            frame(backend, film, task, replay, world, tick // 2)
        previous = world
        world = backend.step(commands)
        for key, sample, speed in backend.last_kick_events:
            replay.on_kick(key, sample, speed)
        for sample in backend.last_substep_worlds:
            replay.observe(sample, previous)
            if replay.success is not None or replay.failure is not None:
                replay.terminal = asdict(sample)
                break
            previous = sample
        if replay.success is not None or replay.failure is not None:
            break
    row = dict(seed=seed, candidate=name, observed_start=start, **replay.result(world))
    row["terminal_state"] = replay.terminal or asdict(world)
    if log:
        log.write(
            json.dumps(
                dict(
                    task=task,
                    profile=profile,
                    passer=passer,
                    scenario=asdict(drill),
                    result=row,
                    frames=ticks,
                ),
                allow_nan=False,
                separators=(",", ":"),
            )
            + "\n"
        )
    if film:
        last_frame = tick // 2 + 1
        frame(backend, film, task, replay, world, last_frame)
        if row["success_s"] is not None and (drill.outlet or drill.context == "moving"):
            for step in range(1, 13):
                commands = replay.commands(world) if drill.context == "moving" else {}
                world = backend.step(commands)
                if step % 2 == 0:
                    frame(backend, film, task, replay, world, last_frame + step // 2)
    return row


def frame(backend, film, task, episode, world, index):
    supervisor = backend.supervisor
    title = task.replace("_", " ").title() + " | " + episode.drill.context
    point = (
        episode.drill.arrival.point
        if episode.drill.arrival
        else episode.drill.outlet or episode.target
    )
    message = f"{world.time:.2f} s"
    if point is not None:
        message += f" | Target ({point_text(point)}) m"
    marker = supervisor.getFromDef("DOJO_TARGET").getField("translation")
    marker.setSFVec3f([*point, 0.003] if point is not None else [0, 0, -1])
    if episode.drill.arrival:
        actual = math.hypot(*vector(world.robot(ACTOR)))
        requested = math.hypot(*episode.drill.arrival.velocity)
        message += f" | {actual:.2f} / {requested:.2f} m/s"
    elif episode.events and episode.events[0]["kind"] == "kick":
        event = episode.events[0]
        who = event["who"]
        verb = "serve" if task == "intercept" else "kick"
        message += f" | {who['team'][0].upper()}{who['id']} {verb} {event['speed']:.2f} m/s"
    if episode.success is not None:
        message += " | SUCCESS"
    elif episode.failure is not None:
        message += f" | {episode.failure.replace('_', ' ')}"
    supervisor.setLabel(0, title, 0.035, 0.033, 0.082, 0xFFFFFF, 0, "Arial")
    supervisor.setLabel(1, message, 0.035, 0.105, 0.060, 0xFFFFFF, 0, "Arial")
    supervisor.exportImage(str(film / f"{index:03d}.jpg"), 84)


def frame_camera(supervisor, drill):
    points = [xy(robot) for robot in drill.world.robots]
    if drill.arrival:
        points.append(drill.arrival.point)
    else:
        points.append(xy(drill.world.ball))
        if drill.outlet:
            points.append(drill.outlet)
        if drill.run_speed:
            receiver = drill.world.robot(RECEIVER)
            points.append((receiver.x, receiver.y + 1.8 * drill.run_speed))
    x, y = zip(*points, strict=True)
    width = max(max(x) - min(x) + 1.15, (max(y) - min(y) + 1.45) * 1.78)
    camera = supervisor.getFromDef("CAMERA")
    camera.getField("position").setSFVec3f(
        [(min(x) + max(x)) / 2, (min(y) + max(y)) / 2, max(4.8, width / 0.81)]
    )
    axis = 1 / math.sqrt(3)
    camera.getField("orientation").setSFRotation([-axis, axis, axis, 2 * math.pi / 3])


def stats(rows):
    success = [r["success_s"] for r in rows if r["success_s"] is not None]
    failures = {}
    for r in rows:
        if r["failure"] is not None:
            failures[r["failure"]] = failures.get(r["failure"], 0) + 1
    return dict(
        trials=len(rows),
        successes=len(success),
        mean_return=mean(r["return_value"] for r in rows),
        median_success_s=median(success) if success else None,
        opponent_first=sum(
            r["defender_touch_s"] is not None
            and (r["first_touch_s"] is None or r["defender_touch_s"] < r["first_touch_s"])
            for r in rows
        ),
        body_contacts=sum(r["body_contact_s"] is not None for r in rows),
        failures=failures,
        median_release_to_control_s=median(
            r["success_s"] - r["release_s"]
            for r in rows
            if r["success_s"] is not None and r["release_s"] is not None
        )
        if any(r["success_s"] is not None and r["release_s"] is not None for r in rows)
        else None,
    )


def paired(backend, task, seeds, choose, passer, log=None, *, probe_cache=None):
    groups = {}
    for index, seed in enumerate(seeds):
        profiles = list(choose(seed).items())
        if index % 2:
            profiles.reverse()
        starts = []
        for arm, (name, knobs) in profiles:
            cache_key = seed, name
            if probe_cache is not None and cache_key in probe_cache:
                row = probe_cache[cache_key]
            else:
                row = trial(backend, task, seed, name, knobs, passer(seed), log)
                if probe_cache is not None:
                    probe_cache[cache_key] = row
            groups.setdefault(arm, []).append(row)
            starts.append(row["observed_start"])
        if any(start != starts[0] for start in starts):
            raise RuntimeError(f"Webots starts differ for {task} / {seed}")
    return groups


def relay_audit(backend, seed):
    drill = scenario("intercept", seed)
    world = backend.reset(
        replace(drill.world, robots=tuple(r for r in drill.world.robots if r.key in SERVERS))
    )
    next_server = SERVERS[0]
    events = []
    for _ in range(round(12 / CFG.dt)):
        world = backend.step(serve_relay(world, next_server, drill.serve_speed))
        for key, observed, speed in backend.last_kick_events:
            if key != next_server:
                raise RuntimeError("Serve relay kicked out of order")
            events.append(
                dict(server=asdict(key), point=xy(observed.ball), time=observed.time, speed=speed)
            )
            next_server = SERVERS[1] if key == SERVERS[0] else SERVERS[0]
        if len(events) >= 4:
            break
    if len(events) != 4:
        raise RuntimeError(f"Serve relay stopped after {len(events)} launches")
    return events


def candidates_for(task, context):
    profiles = PROFILES[task]
    if task != "intercept":
        return profiles
    return {
        name: profile
        for name, profile in profiles.items()
        if name == "baseline" or ("station" in profile) == (context == "deflect")
    }


def train(backend, dest, task_names):
    seed = int(os.environ["ROBOTURK_SEED"])
    episodes = int(os.environ["ROBOTURK_EPISODES"])
    stages = [int(n) for n in os.environ["ROBOTURK_DOJO_STAGES"].split(",")]
    if stages != sorted(set(stages)) or min(stages) < 2 or max(stages) >= 1000:
        raise ValueError("Training start counts must be increasing and between 2 and 999")
    pass_profiles = {
        "to feet": PROFILES["pass"]["baseline"],
        "on the run": PROFILES["pass"]["full_lead"],
    }
    if task_names == ("receive_race",) and os.environ.get("ROBOTURK_CHECKPOINT"):
        pass_model = json.loads(Path(os.environ["ROBOTURK_CHECKPOINT"]).read_text())["tasks"][
            "pass"
        ]
        pass_profiles = {
            ctx: pass_model["profiles"][name] for ctx, name in pass_model["selected"].items()
        }
    report = dict(
        backend=backend.name,
        basic_step_ms=backend.basic_ms,
        control_period_s=CFG.dt,
        train_seed=seed,
        probe_seed=seed + 1000,
        held_out_seed=seed + 2000,
        tasks={},
    )
    for task in task_names:
        candidates = PROFILES[task]
        contexts = [scenario(task, s).context for s in (0, 1)]
        relay = relay_audit(backend, seed) if task == "intercept" else None

        def passer(start):
            if task == "receive_race":
                return pass_profiles["on the run"]
            return {}

        with gzip.open(
            dest.with_name(f"{dest.stem}-{task}.jsonl.gz"), "wt", encoding="utf-8", compresslevel=1
        ) as log:
            samples = paired(
                backend,
                task,
                range(seed, seed + max(stages)),
                lambda start: {
                    n: (n, p)
                    for n, p in candidates_for(task, scenario(task, start).context).items()
                },
                passer,
                log,
            )
        print(
            f"{task}: collected {sum(map(len, samples.values()))} native training episodes",
            flush=True,
        )

        def choose(start):
            return {
                arm: (name, candidates[name])
                for arm, name in {
                    "baseline": "baseline",
                    "fitted": selected[scenario(task, start).context],
                }.items()
            }

        learning = []
        probe_cache = {}
        for n in stages:
            selected = {
                context: max(
                    candidates_for(task, context),
                    key=lambda name: round(
                        mean(
                            r["return_value"]
                            for r in samples[name]
                            if (r["context"] == context or task == "receive_race")
                            and r["seed"] < seed + n
                        ),
                        4,
                    ),
                )
                for context in contexts
            }
            probe = paired(
                backend,
                task,
                range(seed + 1000, seed + 1000 + min(episodes, 24)),
                choose,
                passer,
                probe_cache=probe_cache,
            )
            learning.append(
                dict(
                    starts=n,
                    rollouts=sum(r["seed"] < seed + n for rows in samples.values() for r in rows),
                    selected=selected,
                    probe={arm: stats(rows) for arm, rows in probe.items()},
                )
            )
            print(
                f"{task}: probe at {n} starts: {learning[-1]['probe']['fitted']['successes']} successes",
                flush=True,
            )

        def qualification_profiles(start):
            choices = choose(start)
            if task == "receive_race":
                choices["post_release"] = "early", candidates["early"]
            return choices

        held = paired(
            backend,
            task,
            range(seed + 2000, seed + 2000 + episodes),
            qualification_profiles,
            passer,
        )
        report["tasks"][task] = dict(
            train_seed=seed,
            probe_seed=seed + 1000,
            held_out_seed=seed + 2000,
            profiles=candidates,
            relay_audit=relay,
            learning=learning,
            selected=selected,
            fixed_race_passer=passer(seed),
            qualification={arm: stats(rows) for arm, rows in held.items()},
            by_context={
                ctx: {
                    arm: stats([r for r in rows if r["context"] == ctx])
                    for arm, rows in held.items()
                    if any(r["context"] == ctx for r in rows)
                }
                for ctx in contexts
            },
            held_out=held,
        )
        if task == "pass":
            pass_profiles = {ctx: candidates[name] for ctx, name in selected.items()}
        dest.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(task, selected, report["tasks"][task]["qualification"], flush=True)


def demo(backend, dest, task_names, report):
    backend.supervisor.getRoot().getField("children").importMFNodeFromString(
        -1,
        "DEF DOJO_TARGET Pose { translation 0 0 -1 children [ Shape { appearance PBRAppearance { baseColor 0.47 1 0.82 emissiveColor 0.32 0.60 0.44 transparency 0.30 } geometry Cylinder { radius 0.055 height 0.003 } } ] }",
    )
    results = {}
    for task in task_names:
        item = report["tasks"][task]
        results[task] = {}
        for context in item["by_context"]:
            pairs = zip(item["held_out"]["baseline"], item["held_out"]["fitted"], strict=True)
            options = [
                (a, b) for a, b in pairs if b["context"] == context and b["success_s"] is not None
            ]
            if not options:
                continue
            folder = dest.parent / f"{task}-{context.replace(' ', '-')}"
            options.sort(key=lambda pair: (pair[0]["success_s"] is not None, pair[1]["seed"]))
            folder.mkdir(parents=True, exist_ok=True)
            attempts = []
            for _, selected in options[:4]:
                attempt = Path(tempfile.mkdtemp(prefix=f"{selected['seed']}-", dir=folder))
                name = selected["candidate"]
                result = trial(
                    backend,
                    task,
                    selected["seed"],
                    name,
                    item["profiles"][name],
                    item["fixed_race_passer"],
                    film=attempt,
                )
                attempts.append(
                    dict(
                        seed=selected["seed"],
                        qualification_s=selected["success_s"],
                        replay_s=result["success_s"],
                        failure=result["failure"],
                    )
                )
                if result["success_s"] is not None and math.isclose(
                    result["success_s"], selected["success_s"], abs_tol=CFG.dt + 1e-8
                ):
                    results[task][context] = dict(
                        result, attempts=attempts, frames=str(attempt.relative_to(dest.parent))
                    )
                    break
            else:
                raise RuntimeError(f"Native replay diverged on {task} / {context}: {attempts}")
        dest.write_text(json.dumps(results, indent=2, allow_nan=False) + "\n")


def run(supervisor, mode):
    dest = Path(os.environ["ROBOTURK_OUTPUT"])
    dest.parent.mkdir(parents=True, exist_ok=True)
    backend = ContinuousWebotsBackend(supervisor, CFG)
    selection = os.environ["ROBOTURK_DOJO_TASK"]
    task_names = TASKS if selection == "all" else (selection,)
    try:
        if mode == "primitive-dojo":
            train(backend, dest, task_names)
        else:
            saved = json.loads(Path(os.environ["ROBOTURK_CHECKPOINT"]).read_text())
            if selection == "all":
                task_names = tuple(task for task in TASKS if task in saved["tasks"])
            if mode == "primitive-dojo-transfer":
                from primitive_dojo_transfer import audit

                audit(backend, dest, task_names, saved)
            else:
                demo(backend, dest, task_names, saved)
    except Exception:
        traceback.print_exc()
        dest.with_suffix(".error.txt").write_text(traceback.format_exc())
        supervisor.simulationQuit(1)
    else:
        supervisor.simulationQuit(0)
