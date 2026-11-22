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
from roboturk.five_play import commands
from roboturk.five_roles import Assignment, CFG, TEAMS, FiveRoles, kickoff
from roboturk.model import Referee
from roboturk.ssl_referee import RefereeSocket, RestartGate
from roboturk.three_match import filter_commands, snapshot


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


def match(sim, folder, cfg, receiver=None):
    root = Path(cfg["root"])
    weights = json.loads((root / "policies/recipient_3v3_v1/selectors.json").read_text())
    weights = {t: np.array(weights[str(901 + i)]) for i, t in enumerate(TEAMS)}
    seed = cfg["seed"]
    sign = 1 if seed % 2 == 0 else -1
    serving = TEAMS[(seed // 2) % 2]
    backend = ContinuousWebotsBackend(sim, CFG)
    w = backend.reset(kickoff(seed, serving, sign))
    keys = [r.key for r in w.robots]
    tactics = FiveRoles()
    gate = RestartGate()
    previous_referee = None
    prepared_kickoff = None
    steps = []
    states = []
    events = []
    score = [0, 0]
    restarts = 0
    start_time = sim.getTime()
    if cfg["record"]:
        sim.movieStartRecording(str(folder / "native.mp4"), 1280, 720, 0, 85, 1, False)
    for tick in range(round(cfg["seconds"] / CFG.dt)):
        state = receiver.poll() if receiver else None
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
                w = backend.reset(kickoff(seed + 101 * restarts, serving, sign, w.time))
                prepared_kickoff = kickoff_key
            backend.world = w = replace(w, referee=state.mode, attack_sign=sign)
            score[:] = state.score
            restricted = gate.restricted_to(state.source, state.counter, state.restart_team, w.ball)
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
        if w.referee == Referee.RUNNING:
            sole = tactics.observe(w)
            roles = {}
            phases = {}
            for team in TEAMS:
                phase, side = tactics.assign(w, team)
                phases[team] = phase
                roles.update(side)
            out, planned = commands(w, roles, weights, sole, tactics)
            out, _ = filter_commands(w, out)
            if restricted:
                out = {k: c for k, c in out.items() if k.team == restricted}
        else:
            roles = {r.key: Assignment("hold", (r.x, r.y)) for r in w.robots}
            phases = dict.fromkeys(TEAMS, w.referee.value.upper())
            out, planned = {}, {}
        if not cfg["record"] or receiver:
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
        steps.append(meta)
        states.append(snapshot(w))
        w = backend.step(out)
        for k, _, speed in backend.last_kick_events:
            event = dict(tick=tick, event="shot", actor=keys.index(k), speed=speed)
            if k in planned:
                tactics.flight = planned[k]
                event.update(event="pass", receiver=keys.index(planned[k].receiver))
            else:
                tactics.flight = None
            events.append(event)
        if receiver:
            continue  # The external match official owns goals, scores and restarts.
        scorer = None
        for sample in backend.last_substep_worlds:
            b = sample.ball
            if abs(b.x) > 4.5 + CFG.ball_radius or abs(b.y) > 3 + CFG.ball_radius:
                if (
                    abs(b.x) > 4.5 + CFG.ball_radius
                    and abs(b.y) < CFG.goal_width / 2 - CFG.ball_radius
                ):
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
                w = backend.reset(
                    kickoff(seed + 101 * restarts, serving, sign, (tick + 1) * CFG.dt)
                )
                tactics = FiveRoles()
                break
    if cfg["record"]:
        sim.movieStopRecording()
        deadline = time.monotonic() + 90
        while not sim.movieIsReady() and time.monotonic() < deadline:
            sim.step(40)
        if not sim.movieIsReady() or sim.movieFailed():
            raise RuntimeError("Webots recording failed")
    if folder is not None:
        log = dict(
            seed=seed,
            dt=CFG.dt,
            keys=[[k.team, k.id] for k in keys],
            sign=sign,
            score=score,
            frames=steps,
            events=events,
            seconds=cfg["seconds"],
        )
        (folder / "roles.json.gz").write_bytes(
            gzip.compress(json.dumps(log, separators=(",", ":")).encode(), mtime=0)
        )
        np.savez_compressed(folder / "positions.npz", states=np.array(states))


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
    with connection as receiver:
        match(sim, folder, cfg, receiver)
    if folder:
        sim.simulationQuit(0)
    else:
        sim.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
except Exception:
    if folder:
        (folder / "error.txt").write_text(traceback.format_exc())
    traceback.print_exc()
    sim.simulationQuit(1)
