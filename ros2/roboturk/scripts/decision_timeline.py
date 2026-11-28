"""Store a Hoca timeline and run its candidate from a recorded open-play state."""

import gzip
import hashlib
import json
import os
from datetime import datetime, timezone
from pathlib import Path
import shutil
import signal
import subprocess
import sys
from threading import Lock, Thread
import time
from uuid import uuid4

from controller_versions import load_catalog, match_releases, source_build
from decision_changes import STACK, create_candidate, git, validate_request
from roboturk.five_roles import CFG


TRIAL_SECONDS = 8
RUNTIME_PATHS = ("scripts", "src", "policies")


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
            list(key) + list(positions[4 + 3 * i : 7 + 3 * i]) + velocity
            for i, (key, velocity) in enumerate(zip(log["keys"], velocities, strict=True))
        ],
        "ball": list(positions[:4]) + [frame["ball_z"], frame["ball_vz"]],
        "sign": frame.get("attack_sign", log.get("sign", 1)),
        "owner": frame.get("control"),
        "score": frame["score"],
        "options": {key: config[key] for key in options if key in config} | {"team_limits": limits},
    }


def branch_replay(parent, timeline, trial=None, *, streaming=False):
    tick = timeline["fork_tick"]
    origin = parent["frames"][tick]
    take = tick if trial or streaming else tick + 1
    frames = parent["frames"][:take]
    positions = parent["positions"][:take]
    events = [event for event in parent["events"] if event[0] < tick]
    if trial:
        if parent["keys"] != trial["keys"] or parent["dt"] != trial["dt"]:
            raise ValueError("The Webots trial uses a different replay format.")
        for frame in trial["frames"]:
            stage = (
                "Continued match"
                if timeline.get("promoted") and frame["t"] >= TRIAL_SECONDS
                else "Branch trial"
            )
            frames.append(dict(frame, t=round(origin["t"] + frame["t"], 6), stage=stage))
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
    elif not streaming:
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


def candidate_controllers(record, manifest, build, strategies):
    teams = match_releases(load_catalog(), None, strategies)
    target = record.get("team", "blue")
    teams[target] = dict(
        teams[target],
        name=record.get("controller", {}).get("name", record["title"]),
        version=manifest.get("candidate_version"),
        implementation=manifest.get("implementation")
        or record.get("controller", {}).get("implementation")
        or teams[target]["implementation"],
        candidate=True,
    )
    return {"build": build, "teams": teams}


class DecisionTimelines:
    def __init__(self, root, webots, stack=STACK):
        self.root = root / "decision-timelines"
        self.webots = webots
        self.stack = stack
        self.lock = Lock()
        self.running = None
        self.process = None
        self.installed_controllers = None

    def controllers(self):
        if self.installed_controllers is None:
            catalog = load_catalog()
            self.installed_controllers = {}
            for release_id in catalog["installed"]:
                release = catalog["releases"][release_id]
                try:
                    date = git(self.stack, "show", "-s", "--format=%cI", release["introduced_by"])
                except ValueError:
                    date = None
                self.installed_controllers[release_id] = dict(
                    id=release_id,
                    name=release["name"],
                    version=release["version"],
                    implementation=release["implementation"],
                    created_at=date,
                    updated_at=date,
                )
        saved = (
            json.loads(path.read_text()) for path in (self.root / "controllers").glob("*.json")
        )
        return self.installed_controllers | {release["id"]: release for release in saved}

    def save_controller(self, record):
        build = record.get("build")
        identity = record.get("controller")
        if not identity or not build or build.get("working_copy_sha256"):
            return
        team = record["controller_versions"]["teams"][record["team"]]
        commit = build["commit"]
        version = dict(
            identity,
            id=f"{identity['id']}@{commit[:12]}",
            commit=commit,
            version=team["version"] or commit[:8],
            implementation=team["implementation"],
            updated_at=git(self.stack, "show", "-s", "--format=%cI", commit),
        )
        root = self.root / "controllers"
        root.mkdir(parents=True, exist_ok=True)
        key = hashlib.sha256(version["id"].encode()).hexdigest()[:20]
        path = root / f"{key}.json"
        pending = path.with_suffix(".tmp")
        pending.write_text(json.dumps(version, indent=2) + "\n")
        pending.replace(path)

    def directory(self, source):
        return self.root / hashlib.sha256(source.encode()).hexdigest()[:20]

    def read(self, source):
        path = self.directory(source) / "timeline.json"
        if not path.is_file():
            return None
        record = json.loads(path.read_text())
        attached = (
            self.running == source and self.process is not None and self.process.poll() is None
        )
        if (
            record["status"] in ("running", "switching", "continuing")
            and not attached
            and self.running != source
        ):
            record["status"] = "failed"
        return dict(record, resumable=attached and record["status"] == "ready")

    def save(self, source, record):
        path = self.directory(source) / "timeline.json"
        pending = path.with_suffix(".tmp")
        pending.write_text(
            json.dumps({k: v for k, v in record.items() if k != "resumable"}, indent=2) + "\n"
        )
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
            parent_id = request.get("controller_base")
            parent = self.controllers().get(parent_id) if parent_id else None
            if parent_id and parent is None:
                raise ValueError("Choose an available controller version to build on.")
            candidate = create_candidate(
                request, stack=self.stack, base=parent.get("commit", "HEAD") if parent else "HEAD"
            )
            now = datetime.now(timezone.utc).isoformat(timespec="seconds")
            record = {key: value for key, value in candidate.items() if key != "created"} | {
                "source": source,
                "title": request["title"],
                "team": request["team"],
                "parent_timeline": "main",
                "fork_tick": tick,
                "fork_time": time,
                "status": "awaiting",
                "run": None,
                "trial_end": time,
                "controller": {
                    "id": candidate["branch"],
                    "name": parent["name"]
                    if parent
                    else request.get("controller_name", "").strip() or request["title"],
                    "created_at": parent["created_at"] if parent else now,
                    "updated_at": now,
                    "parent_id": parent_id,
                    "implementation": parent["implementation"] if parent else None,
                },
            }
            self.directory(source).mkdir(parents=True, exist_ok=True)
            self.save(source, record)
            return dict(record, resumable=False), True

    def run(self, source, log, samples, config, duration):
        with self.lock:
            record = self.read(source)
            if not record or record.get("promoted"):
                raise ValueError("Open a candidate timeline that has yet to take over.")
            if self.running is not None:
                active = self.read(self.running)
                if active["status"] != "ready":
                    raise ValueError("Wait for the active branch to finish.")
                self.stop(active)
            initial = fork_input(log, samples, record["fork_tick"], config)
            initial["preview_seconds"] = TRIAL_SECONDS
            stack = Path(record["worktree"]) / self.stack.relative_to(
                Path(git(self.stack, "rev-parse", "--show-toplevel"))
            )
            runner = stack / "scripts/run_five_roles.py"
            if not runner.is_file():
                raise ValueError("The candidate workspace is unavailable.")
            build = source_build(stack, RUNTIME_PATHS)
            manifest = json.loads(Path(record["record"]).read_text())
            if record.get("controller") and build and not build["working_copy_sha256"]:
                record["controller"]["updated_at"] = git(
                    stack, "show", "-s", "--format=%cI", build["commit"]
                )
            options = initial["options"]
            versions = candidate_controllers(
                record, manifest, build, options.get("team_strategies")
            )
            target = record.get("team", "blue")
            strategies = options.get("team_strategies") or {
                team: {"strategy": "balanced", "pass_preference": 0} for team in ("blue", "yellow")
            }
            strategies[target]["controller"] = versions["teams"][target]["implementation"]
            options["team_strategies"] = strategies
            run = self.directory(source) / uuid4().hex[:12]
            run.mkdir()
            spec = run / "fork.json"
            spec.write_text(json.dumps(initial, allow_nan=False))
            status = dict(
                record,
                status="running",
                run=run.name,
                build=build,
                trial_end=record["fork_time"],
                controller_versions=versions,
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
                str(max(TRIAL_SECONDS + 1, duration)),
                "--live-viewer",
            ]
            if self.webots:
                command.extend(["--webots", self.webots])
            with (run / "launch.log").open("w") as output:
                process = subprocess.Popen(
                    command, stdout=output, stderr=subprocess.STDOUT, start_new_session=True
                )
            self.running, self.process = source, process
            try:
                self.save(source, status)
            except OSError:
                self.stop(status)
                raise
            Thread(target=self.monitor, args=(source, process, status), daemon=True).start()
            return status

    def monitor(self, source, process, status):
        folder = self.trial_folder(status)
        ready = folder / "preview-ready"
        until = time.monotonic() + 55
        while process.poll() is None and not ready.is_file() and time.monotonic() < until:
            time.sleep(0.05)
        with self.lock:
            if self.process is not process:
                return
            if ready.is_file() and process.poll() is None:
                self.save(source, self.finished_record(status, folder / "preview", "ready"))
                self.save_controller(status)
            elif process.poll() is None:
                self.stop(status)
                self.save(source, dict(status, status="failed"))
                return
        process.wait()
        with self.lock:
            if self.process is not process:
                return
            latest = self.read(source)
            try:
                if process.returncode == 0 and latest.get("promoted"):
                    self.save(source, self.finished_record(latest, folder, "finished"))
                elif process.returncode == 0 and (folder / "roles.json.gz").is_file():
                    self.save(source, self.finished_record(latest, folder, "ready"))
                elif latest["status"] != "ready":
                    self.save(source, dict(latest, status="failed"))
            finally:
                self.running, self.process = None, None

    @staticmethod
    def finished_record(status, folder, phase):
        with gzip.open(folder / "roles.json.gz", "rt") as roles:
            trace = json.load(roles)
        return dict(
            status,
            status=phase,
            trial_end=round(status["fork_time"] + trace["frames"][-1]["clock"], 6),
        )

    def promote(self, source, stop_parent):
        with self.lock:
            record = self.read(source)
            if not record or not record["resumable"]:
                raise ValueError(
                    "Run the trial and wait for Webots to pause at the end of the preview."
                )
            build = record["build"]
            manifest = json.loads(Path(record["record"]).read_text())
            if (
                not build
                or build["working_copy_sha256"]
                or build["commit"] == manifest["base_commit"]
            ):
                raise ValueError(
                    "Commit the controller change and run a trial of that commit before continuing."
                )
            master = git(self.stack, "symbolic-ref", "--short", "HEAD")
            if master not in ("main", "master"):
                raise ValueError("Open Hoca from the main checkout to verify the merge.")
            try:
                git(self.stack, "merge-base", "--is-ancestor", build["commit"], "HEAD")
            except ValueError:
                if git(self.stack, "status", "--porcelain"):
                    raise ValueError(
                        "Finish the local edits on main before merging this trial."
                    ) from None
                try:
                    git(self.stack, "merge", "--ff-only", build["commit"])
                except ValueError:
                    raise ValueError(
                        "Main has diverged. Merge the tested commit, then return to Hoca."
                    ) from None
            self.save(source, dict(record, status="switching"))
            try:
                stop_parent()
                promoted = dict(record, status="continuing", promoted=True)
                self.save(source, promoted)
                (self.trial_folder(record) / "continue-branch").touch()
            except (OSError, ValueError):
                self.save(source, record)
                raise
            return promoted

    def stop(self, record):
        process = self.process
        if process is None:
            return
        if process.poll() is None:
            folder = self.trial_folder(record)
            try:
                (folder / "stop-branch").touch()
                process.wait(timeout=3)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
                except ProcessLookupError:
                    process.wait()
                except subprocess.TimeoutExpired:
                    raise ValueError("Webots is still shutting down. Try again shortly.") from None
        self.running, self.process = None, None

    def close(self):
        with self.lock:
            if self.running:
                self.stop(self.read(self.running))

    def trial_folder(self, record):
        return self.directory(record["source"]) / record["run"] / "match"


def webots_path(spec):
    if spec.get("webots"):
        return spec["webots"]
    runtime = Path(os.environ.get("ROBOTURK_RUNTIME", STACK.parents[2] / "runtime"))
    bundled = runtime / "Webots-nightly.app/Contents/MacOS/webots"
    return shutil.which("webots") or (str(bundled) if bundled.is_file() else None)
