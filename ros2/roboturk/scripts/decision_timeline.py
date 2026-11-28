"""Store a Hoca timeline and run its candidate from a recorded open-play state."""

import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from threading import Lock, Thread
from uuid import uuid4

from controller_versions import source_build
from decision_changes import STACK, create_candidate, validate_request
from roboturk.five_roles import CFG


TRIAL_SECONDS = 8


def fork_input(log, samples, tick, config):
    frame = log["frames"][tick]
    if (
        frame.get("mode", "running") != "running"
        or frame.get("restricted_to")
        or frame.get("placer") is not None
        or any(phase in ("Kickoff", "Restart") for phase in frame.get("phases", []))
    ):
        raise ValueError("Choose an open-play frame after the restart.")
    allocation = frame.get("allocation") or []
    velocities = [item.get("measured_velocity") for item in allocation]
    if (
        len(velocities) != len(log["keys"])
        or any(not v or len(v) != 3 for v in velocities)
        or len(log["keys"]) != 10
        or log["dt"] != CFG.dt
        or frame.get("ball_z") is None
        or frame.get("ball_vz") is None
    ):
        raise ValueError("This trial requires a 5v5 frame with recorded robot velocities.")
    positions = samples[tick]
    limits = next(
        (item["limits"] for item in reversed(log["frames"][: tick + 1]) if item.get("limits")),
        log.get("limits"),
    )
    options = (
        "team_strategies",
        "receive_profile",
        "open_play_policy",
        "smooth_carry",
        "tackle_policy",
        "tackle_selector",
        "keeper_cover",
        "keeper_timing",
        "keeper_lane",
        "pivot_fetch",
        "trajectory_planner",
        "errt_fallback",
        "commit_waypoints",
        "release_waypoint_early",
        "touchline_recovery",
        "crowded_fetch",
        "spin_releases",
        "pass_intercepts",
    )
    return {
        "robots": [
            list(key) + positions[4 + 3 * i : 7 + 3 * i].tolist() + velocity
            for i, (key, velocity) in enumerate(zip(log["keys"], velocities, strict=True))
        ],
        "ball": positions[:4].tolist() + [frame["ball_z"], frame["ball_vz"]],
        "sign": frame.get("attack_sign", log.get("sign", 1)),
        "owner": frame.get("control"),
        "score": frame["score"],
        "options": {key: config[key] for key in options if key in config} | {"team_limits": limits},
    }


def branch_replay(parent, timeline, trial=None):
    tick = timeline["fork_tick"]
    origin = parent["frames"][tick]
    take = tick if trial else tick + 1
    frames = parent["frames"][:take]
    positions = parent["positions"][:take]
    events = [event for event in parent["events"] if event[0] < tick]
    if trial:
        if parent["keys"] != trial["keys"] or parent["dt"] != trial["dt"]:
            raise ValueError("The Webots trial uses a different replay format.")
        for frame in trial["frames"]:
            frames.append(dict(frame, t=round(origin["t"] + frame["t"], 6), stage="Branch trial"))
        positions.extend(trial["positions"])
        for index, label, kind, team, *extra in trial["events"]:
            if extra and extra[0]:
                extra = [
                    extra[0]
                    | {
                        field: extra[0][field] + tick
                        for field in ("tick", "kick_tick")
                        if field in extra[0]
                    }
                ]
            events.append([tick + index, label, kind, team, *extra])
    else:
        # The coach is changing this decision. The parent event at the fork belongs to its future.
        frames[-1] = dict(origin, stage="Branch point")
    return dict(
        parent,
        frames=frames,
        positions=positions,
        events=events,
        forecast=None,
        intro=dict(parent["intro"], frame=tick),
        active_timeline="branch",
    )


class DecisionTimelines:
    def __init__(self, root, webots):
        self.root = root / "decision-timelines"
        self.webots = webots
        self.lock = Lock()
        self.running = None

    def directory(self, source):
        return self.root / hashlib.sha256(source.encode()).hexdigest()[:20]

    def read(self, source):
        path = self.directory(source) / "timeline.json"
        if not path.is_file():
            return None
        record = json.loads(path.read_text())
        if record["status"] == "running" and self.running != source:
            return dict(record, status="failed")
        return record

    def save(self, source, record):
        path = self.directory(source) / "timeline.json"
        pending = path.with_suffix(".tmp")
        pending.write_text(json.dumps(record, indent=2) + "\n")
        pending.replace(path)

    def create(self, source, request, log):
        validate_request(request)
        clip = request["evidence"]
        tick = clip["review"]["tick"]
        if tick >= len(log["frames"]) or clip["source"].get("seed") != log["seed"]:
            raise ValueError("Choose a decision frame from this recording.")
        time = log["frames"][tick].get("clock", round(tick * log["dt"], 4))
        with self.lock:
            existing = self.read(source)
            if existing:
                saved = Path(existing["record"]).with_name("request.json")
                if saved.is_file() and request == json.loads(saved.read_text()):
                    return existing, False
                raise ValueError("This match already has a candidate timeline.")
            candidate = create_candidate(request)
            record = {key: value for key, value in candidate.items() if key != "created"} | {
                "source": source,
                "title": request["title"],
                "parent_timeline": "main",
                "fork_tick": tick,
                "fork_time": time,
                "status": "awaiting",
                "run": None,
                "trial_end": time,
            }
            self.directory(source).mkdir(parents=True, exist_ok=True)
            self.save(source, record)
            return record, True

    def run(self, source, log, samples, config):
        with self.lock:
            record = self.read(source)
            if not record or self.running is not None:
                raise ValueError(
                    "Open a candidate timeline and wait for any running trial to finish."
                )
            initial = fork_input(log, samples, record["fork_tick"], config)
            stack = Path(record["record"]).parents[3]
            runner = stack / "scripts/run_five_roles.py"
            if not runner.is_file():
                raise ValueError("The candidate workspace is unavailable.")
            build = source_build(stack)
            run = self.directory(source) / uuid4().hex[:12]
            run.mkdir()
            spec = run / "fork.json"
            spec.write_text(json.dumps(initial, allow_nan=False))
            status = dict(
                record, status="running", run=run.name, build=build, trial_end=record["fork_time"]
            )
            command = [
                sys.executable,
                str(runner),
                str(run / "match"),
                "--fork-state",
                str(spec),
                "--seed",
                str(log["seed"]),
                "--seconds",
                str(TRIAL_SECONDS),
                "--role-audit",
            ]
            if self.webots:
                command.extend(["--webots", self.webots])
            self.running = source
            try:
                self.save(source, status)
            except OSError:
                self.running = None
                raise
            Thread(target=self.finish, args=(source, run, command, status), daemon=True).start()
            return status

    def finish(self, source, run, command, status):
        result = "failed"
        try:
            with (run / "launch.log").open("w") as output:
                subprocess.run(
                    command, stdout=output, stderr=subprocess.STDOUT, check=True, timeout=65
                )
            with gzip.open(run / "match/roles.json.gz", "rt") as roles:
                trace = json.load(roles)
            status["trial_end"] = round(status["fork_time"] + trace["frames"][-1]["clock"], 6)
            result = "ready"
        except (OSError, ValueError, KeyError, IndexError, subprocess.SubprocessError):
            pass
        finally:
            with self.lock:
                try:
                    self.save(source, dict(status, status=result))
                finally:
                    self.running = None

    def trial_folder(self, record):
        return self.directory(record["source"]) / record["run"] / "match"


def webots_path(spec):
    if spec.get("webots"):
        return spec["webots"]
    runtime = Path(os.environ.get("ROBOTURK_RUNTIME", STACK.parents[2] / "runtime"))
    bundled = runtime / "Webots-nightly.app/Contents/MacOS/webots"
    return shutil.which("webots") or (str(bundled) if bundled.is_file() else None)
