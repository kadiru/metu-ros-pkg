"""Native 5v5 match with an on-field jersey HUD and live role rosters."""

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
from roboturk.five_roles import CFG, TEAMS, FiveRoles, kickoff
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


def match(sim, folder, cfg):
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
    steps = []
    states = []
    events = []
    score = [0, 0]
    restarts = 0
    start_time = sim.getTime()
    if cfg["record"]:
        sim.movieStartRecording(str(folder / "native.mp4"), 1280, 720, 0, 85, 1, False)
    for tick in range(round(cfg["seconds"] / CFG.dt)):
        sole = tactics.observe(w)
        roles = {}
        phases = {}
        for team in TEAMS:
            phase, side = tactics.assign(w, team)
            phases[team] = phase
            roles.update(side)
        out, planned = commands(w, roles, weights, sole, tactics)
        out, _ = filter_commands(w, out)
        if not cfg["record"]:
            hud(sim, w, keys, roles, phases, score, tick * CFG.dt)
        meta = dict(
            clock=round(sim.getTime() - start_time, 6),
            roles=[roles[k].role for k in keys],
            phases=list(phases.values()),
            control=keys.index(tactics.control) if tactics.control else None,
            score=score.copy(),
            restart=restarts,
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
    match(sim, folder, cfg)
    if folder:
        sim.simulationQuit(0)
    else:
        sim.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
except Exception:
    if folder:
        (folder / "error.txt").write_text(traceback.format_exc())
    traceback.print_exc()
    sim.simulationQuit(1)
