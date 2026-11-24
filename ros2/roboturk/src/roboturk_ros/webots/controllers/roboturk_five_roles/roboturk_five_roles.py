"""Native 5v5 match with an on-field jersey HUD and live role rosters."""

from contextlib import closing, nullcontext
from dataclasses import replace
import gzip
import json
import os
from pathlib import Path
import time
import traceback

import numpy as np
from controller import Supervisor
from roboturk.continuous_backend import ContinuousWebotsBackend
from roboturk.field import CARPET_MARGIN
from roboturk.five_play import open_play
from roboturk.five_roles import Assignment, CFG, TEAMS, FiveRoles, kickoff
from roboturk.kickoff_drive import KickoffDrive, ParallelKickoffDrive
from roboturk.team_kickoff import IndependentKickoffs
from roboturk.model import Referee, SSL_KICK_LIMIT
from roboturk.open_motion import Steering, observed_velocity
from roboturk.placement import PlacementRestart
from roboturk.ssl_referee import RefereeSocket, RestartGate
from roboturk.ssl_vision import (
    VisionOutput,
    Wrapper,
    detection,
    practice_command,
    read_delimited,
    write_delimited,
)
from roboturk.three_match import snapshot


def hud(sim, w, keys, roles, phases, score, seconds):
    def label(i, text, x, y, size, color=0xFFFFFF):
        sim.setLabel(i, text, x, y, size, color, 0, "Arial")

    label(
        0,
        f"RoboTurk   5v5       {seconds:05.1f} s       {score[0]} - {score[1]}",
        0.23,
        0.016,
        0.049,
    )
    for i, k in enumerate(keys):
        r = w.robot(k)
        x = 0.018 if i < 5 else 0.84
        color = 0x88BCFF if i < 5 else 0xFFE774
        if k.id == 0:
            label(20 + i, k.team.upper() + "  " + phases[k.team], x, 0.17, 0.041, color)
        label(1 + i, f"{k.id}  {roles[k].role}", x, 0.24 + k.id * 0.073, 0.035, color)
        label(32 + i, str(k.id), 0.5 + r.x * 0.073, 0.485 - r.y * 0.13, 0.030, color)


def match(sim, folder, cfg, receiver=None, vision=None):
    root = Path(cfg["root"])
    weights = json.loads((root / "policies/recipient_3v3_v1/selectors.json").read_text())
    weights = {t: np.array(weights[str(901 + i)]) for i, t in enumerate(TEAMS)}
    seed = cfg["seed"]
    sign = 1 if seed % 2 == 0 else -1
    serving = TEAMS[(seed // 2) % 2]
    backend = ContinuousWebotsBackend(
        sim, replace(CFG, max_kick=SSL_KICK_LIMIT) if receiver else CFG
    )
    drive = cfg.get("drive_kickoffs", False)
    formation = None
    ball_centered = False
    if drive:
        # The league table starts Blue on the negative half and alternates first service.
        w = backend.reset(kickoff(seed + 101, TEAMS[seed % 2], 1))
    else:
        w = backend.reset(kickoff(seed, serving, sign))
    keys = [r.key for r in w.robots]
    tactics = FiveRoles()
    gate = RestartGate()
    restart = (
        PlacementRestart(
            json.loads((root / "policies/placement_v1/actor.json").read_text())["weights"],
            json.loads((root / "policies/placement_v2/actor.json").read_text())["traffic"],
            clear_retreat=True,
        )
        if receiver
        else None
    )
    previous_referee = None
    prepared_kickoff = None
    awaiting_team = None
    saw_restart = False
    finish_tick = None
    steps = []
    states = []
    events = []
    score = [0, 0]
    restarts = 0
    if cfg.get("await_force_start"):
        deadline = time.monotonic() + 6
        while receiver.poll().command != "FORCE_START":
            if time.monotonic() >= deadline:
                raise RuntimeError("No official FORCE_START for paired opening")
            time.sleep(0.01)
    start_time = sim.getTime()
    origin = time.time()
    camera_delay = 0.0
    practice = practice_command(sign)

    def send_camera(world, frame=None):
        packet = (receiver.feed.packet if state.source else None) if receiver else practice
        stamp = origin + (sim.getTime() - start_time) + camera_delay
        if not receiver:
            practice.packet_timestamp = round(stamp * 1e6)
        if frame is not None:
            return vision.send_frame(frame, stamp, packet)
        return vision.send(world, stamp, backend.ball.getPosition()[2], packet)

    policy = Steering(**cfg["open_play_policy"]) if cfg.get("open_play_policy") else None
    previous_positions = {}
    previous_restart = None
    capture_restart = cfg.get("stop_after_placement_reception", False)
    frame_images = cfg.get("frame_images") or (capture_restart and cfg["record"])
    if frame_images:
        (folder / "frames").mkdir()
    elif cfg["record"]:
        sim.movieStartRecording(str(folder / "native.mp4"), 1280, 720, 0, 85, 1, False)
    league = cfg.get("until_post_game", False)
    started = False
    if league:
        (folder / "referee-ready").touch()
    for tick in range(round(cfg["seconds"] / CFG.dt)):
        state = receiver.poll() if receiver else None
        opening = False
        restricted = ""
        if state:
            sign = state.attack_sign
            kickoff_key = (state.source, state.counter)
            if (
                state.command.startswith("PREPARE_KICKOFF_")
                and state.mode == Referee.STOP
                and kickoff_key != prepared_kickoff
            ):
                serving = state.command.split("_")[-1].lower()
                restarts += 1
                target = kickoff(seed + 101 * restarts, serving, sign, w.time)
                if drive and cfg.get("track_kickoffs"):
                    events.append(
                        dict(
                            event="formation_start",
                            restart=restarts,
                            tick=tick,
                            velocities=[[r.vx, r.vy, r.omega] for r in w.robots],
                        )
                    )
                if drive:
                    planner = ParallelKickoffDrive if cfg.get("parallel_kickoffs") else KickoffDrive
                    formation = (
                        IndependentKickoffs(target) if cfg.get("team_kickoffs") else planner(target)
                    )
                    ball_centered = False
                else:
                    w = backend.reset(target)
                prepared_kickoff = kickoff_key
            if league and not started and state.command == "NORMAL_START" and state.restart_team:
                # Equalize settling time at the first authorized action of the paired fixture.
                w = backend.reset(kickoff(seed + 101 * restarts, state.restart_team, sign, w.time))
                previous_positions = {}
                started = opening = True
            backend.world = w = replace(w, referee=state.mode, attack_sign=sign)
            if cfg.get("stop_after_goal_restart") and awaiting_team is None:
                for idx, goals in enumerate(state.score):
                    if goals > score[idx]:
                        awaiting_team = TEAMS[1 - idx]
            score[:] = state.score
            restricted = gate.restricted_to(state.source, state.counter, state.restart_team, w.ball)
            if awaiting_team and state.command == "NORMAL_START":
                if restricted == awaiting_team:
                    saw_restart = True
                elif saw_restart and not restricted and finish_tick is None:
                    finish_tick = tick + round(1.5 / CFG.dt)
            referee_key = (
                state.source,
                state.counter,
                state.command,
                state.stage,
                state.mode,
                sign,
            )
            if referee_key != previous_referee:
                tactics = FiveRoles()
                events.append(
                    dict(
                        tick=tick,
                        event="referee",
                        command=state.command,
                        mode=state.mode.value,
                        stage=state.stage,
                        source=state.source,
                        counter=state.counter,
                        sign=sign,
                    )
                )
                print(f"Referee {state.command or 'NO SIGNAL'} -> {state.mode.value}", flush=True)
                previous_referee = referee_key
        if policy:
            if previous_restart != restarts:
                previous_positions = {}
            measured = observed_velocity(w.robots, previous_positions, CFG.dt)
            previous_positions = {r.key: (r.x, r.y) for r in w.robots}
            previous_restart = restarts
        previous_play = restart.play if restart else None
        placement = restart.commands(w, state) if restart else None
        if previous_play and not restart.play and restart.received >= 5:
            events.append(
                dict(tick=tick, event="restart_received", actor=keys.index(previous_play.receiver))
            )
            if capture_restart:
                finish_tick = tick

        positioning = (
            formation is not None
            and state.command.startswith("PREPARE_KICKOFF_")
            and state.mode == Referee.STOP
        )
        kickoff_frame = None
        if positioning:
            if isinstance(formation, IndependentKickoffs):
                stamp = origin + (sim.getTime() - start_time) + camera_delay
                kickoff_frame = detection(w, tick, stamp, backend.ball.getPosition()[2])
                out, arrived = formation.commands(w, CFG.dt, kickoff_frame.detection)
            else:
                out, arrived = formation.commands(w, CFG.dt)
            if arrived and not ball_centered:
                backend.ball.getField("translation").setSFVec3f([0, 0, CFG.ball_radius])
                backend.ball.resetPhysics()
                ball_centered = True
            elif arrived and abs(w.ball.x) + abs(w.ball.y) < 0.004:
                marker = folder / f"formation-{restarts}-{serving.upper()}.ready"
                if not marker.exists():
                    marker.touch()
                    events.append(dict(tick=tick, event="formation_ready", restart=restarts))
            roles = {k: Assignment("kickoff", (r.x, r.y)) for k, r in formation.targets.items()}
            phases = dict.fromkeys(TEAMS, "Kickoff")
            planned = {}
        elif placement:
            out, roles = placement
            phases = {t: "Restart" for t in TEAMS}
            planned = {}
        elif w.referee == Referee.RUNNING:
            active_policy = policy if tick >= cfg.get("steer_from_tick", 0) else None
            out, planned, roles, phases = open_play(
                w,
                tactics,
                weights,
                measured if active_policy else {},
                active_policy,
                cfg.get("touchline_recovery", True),
            )
            if restricted:
                out = {k: c for k, c in out.items() if k.team == restricted}
        else:
            roles = {r.key: Assignment("hold", (r.x, r.y)) for r in w.robots}
            phases = dict.fromkeys(TEAMS, w.referee.value.upper())
            out, planned = {}, {}
        if (not cfg["record"] or receiver) and not capture_restart and not frame_images:
            hud(sim, w, keys, roles, phases, score, tick * CFG.dt)
            if state:
                sim.setLabel(
                    50,
                    "SSL   " + (state.command or "NO SIGNAL"),
                    0.34,
                    0.075,
                    0.035,
                    0xFFFFFF,
                    0,
                    "Arial",
                )
        meta = dict(
            clock=round(sim.getTime() - start_time, 6),
            roles=[roles[k].role for k in keys],
            phases=list(phases.values()),
            control=keys.index(tactics.control) if tactics.control else None,
            score=score.copy(),
            restart=restarts,
        )
        if state:
            meta.update(
                ssl_command=state.command,
                mode=state.mode.value,
                restricted_to=restricted,
                attack_sign=sign,
            )
        if league:
            packet = receiver.feed.packet
            meta.update(
                stage=state.stage,
                counter=state.counter,
                stage_seconds=packet.stage_time_left / 1e6
                if state.source and packet.HasField("stage_time_left")
                else None,
                first_action=opening,
            )
        if restart and restart.play:
            meta.update(
                placer=keys.index(restart.play.placer),
                receiver=keys.index(restart.play.receiver),
                target=restart.play.target,
                placement_phase=restart.play.phase,
            )
        if cfg.get("motion_audit"):
            meta["ball_z"] = backend.ball.getPosition()[2]
            if tick == 0 or opening:
                meta["opening_velocities"] = [[r.vx, r.vy, r.omega] for r in w.robots]
        if frame_images and tick % 2 == 0:
            sim.exportImage(str(folder / "frames" / f"{tick // 2:05d}.jpg"), 85)
        if vision:
            meta["vision_time"] = origin + (sim.getTime() - start_time) + camera_delay
            if kickoff_frame is not None and cfg.get("track_kickoffs"):
                d = kickoff_frame.detection
                meta["kickoff_camera_mm"] = [
                    [team, b.robot_id, b.x, b.y]
                    for team, bots in (("blue", d.robots_blue), ("yellow", d.robots_yellow))
                    for b in bots
                ]
                meta["kickoff_commands"] = [[out[k].vx, out[k].vy, out[k].omega] for k in keys]
            tracked = send_camera(w, kickoff_frame)
            if cfg.get("track_kickoffs") and tracked is not None:
                meta["tracker_time"] = tracked.timestamp
                meta["tracked_xy"] = [
                    [bot.robot_id.team, bot.robot_id.id, bot.pos.x, bot.pos.y]
                    for bot in tracked.robots
                ]
        steps.append(meta)
        states.append(snapshot(w))
        w = backend.step(out)
        if restart:
            restart.applied(backend.last_kick_events)
        for k, _, speed in backend.last_kick_events:
            event = dict(tick=tick, event="shot", actor=keys.index(k), speed=speed)
            if (
                restart
                and restart.play
                and restart.direct_since is not None
                and k == restart.play.placer
            ):
                event.update(event="restart_pass", receiver=keys.index(restart.play.receiver))
            if k in planned:
                tactics.flight = planned[k]
                event.update(event="pass", receiver=keys.index(planned[k].receiver))
            else:
                tactics.flight = None
            events.append(event)
        if tick == finish_tick or (league and state.stage == "POST_GAME"):
            break
        if receiver:
            continue  # The external match official owns goals, scores and restarts.
        scorer = None
        for sample in backend.last_substep_worlds:
            b = sample.ball
            if abs(b.x) > CFG.ball_exit_x or abs(b.y) > CFG.ball_exit_y:
                if abs(b.x) > CFG.ball_exit_x and abs(b.y) < CFG.goal_width / 2 - CFG.ball_radius:
                    scorer = 0 if b.x * sign > 0 else 1
                    score[scorer] += 1
                serving = (
                    TEAMS[1 - scorer]
                    if scorer is not None
                    else (
                        "yellow" if tactics.control and tactics.control.team == "blue" else "blue"
                    )
                )
                restarts += 1
                events.append(
                    dict(
                        tick=tick,
                        event="goal" if scorer is not None else "out",
                        team=TEAMS[scorer] if scorer is not None else None,
                    )
                )
                if cfg.get("motion_audit"):
                    events[-1]["crossing_xyz"] = [b.x, b.y, backend.ball.getPosition()[2]]
                if cfg.get("capture_exit") == restarts:
                    with (folder / "tail.pb").open("wb") as stream:
                        for index in range(round(1.0 / CFG.dt)):
                            frame = detection(
                                w, index, index * CFG.dt, backend.ball.getPosition()[2]
                            )
                            write_delimited(stream, frame)
                            w = backend.step({})
                    break
                if cfg.get("exit_tails"):
                    path = Path(cfg["exit_tails"]) / str(seed) / str(restarts) / "tail.pb"
                    with path.open("rb") as stream:
                        for index in range(round(1.0 / CFG.dt)):
                            frame = read_delimited(stream, Wrapper)
                            if index == 0:
                                actual = detection(w, 0, 0, backend.ball.getPosition()[2])
                                if actual != frame:
                                    raise ValueError(
                                        f"Exit branch disagrees with live state: {path}"
                                    )
                            stamp = origin + (sim.getTime() - start_time) + camera_delay
                            vision.send_frame(frame, stamp, practice)
                            camera_delay += CFG.dt
                        if stream.read(1):
                            raise ValueError(f"Unexpected trailing camera frames: {path}")
                w = backend.reset(
                    kickoff(seed + 101 * restarts, serving, sign, (tick + 1) * CFG.dt)
                )
                tactics = FiveRoles()
                break
        if cfg.get("capture_exit") == restarts:
            break
    if cfg["record"] and not frame_images:
        sim.movieStopRecording()
        deadline = time.monotonic() + 90
        while not sim.movieIsReady() and time.monotonic() < deadline:
            sim.step(40)
        if not sim.movieIsReady() or sim.movieFailed():
            raise RuntimeError("Webots recording failed")
    if folder is not None:
        log = dict(
            carpet_length=CFG.length + 2 * CARPET_MARGIN,
            seed=seed,
            dt=CFG.dt,
            keys=[[k.team, k.id] for k in keys],
            sign=sign,
            score=score,
            frames=steps,
            events=events,
            seconds=len(steps) * CFG.dt,
        )
        (folder / "roles.json.gz").write_bytes(
            gzip.compress(json.dumps(log, separators=(",", ":")).encode(), mtime=0)
        )
        np.savez_compressed(folder / "positions.npz", states=np.array(states))
        if league and (not started or state.stage != "POST_GAME"):
            raise RuntimeError("Fixture ended before official kickoff or POST_GAME")


sim = Supervisor()
output = os.environ.get("FIVE_ROLE_RUN")
folder = Path(output) if output else None
cfg = (
    json.loads((folder / "config.json").read_text())
    if folder
    else dict(
        root=str(Path(__file__).resolve().parents[5]), seed=20300000, seconds=90, record=False
    )
)
try:
    live = cfg.get("referee") == "ssl"
    connection = (
        closing(
            RefereeSocket(cfg["referee_address"], cfg["referee_port"], cfg["referee_interface"])
        )
        if live
        else nullcontext()
    )
    camera = (
        closing(VisionOutput(cfg.get("vision"), cfg.get("autoref_ci"), cfg.get("tracker")))
        if cfg.get("vision") or cfg.get("autoref_ci")
        else nullcontext()
    )
    with connection as receiver, camera as vision:
        if cfg.get("touchline_trials"):
            from touchline_trials import run

            run(sim, folder, cfg)
        elif cfg.get("kickoff_replays") is not None:
            from kickoff_replay import run

            run(
                sim,
                folder,
                cfg["kickoff_replays"],
                teams=cfg.get("team_kickoffs", False),
                film=cfg.get("frame_images", False),
            )
        elif cfg.get("placement_batch"):
            from placement_runner import run

            run(sim, folder, cfg, receiver, vision)
        else:
            match(sim, folder, cfg, receiver, vision)
    if folder:
        sim.simulationQuit(0)
    else:
        sim.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
except Exception:
    if folder:
        (folder / "error.txt").write_text(traceback.format_exc())
    traceback.print_exc()
    sim.simulationQuit(1)
