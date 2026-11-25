"""Validate spectator setup and launch one local Webots match at a time."""

from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from threading import Lock, Thread
from uuid import uuid4

from roboturk.five_strategy import decode_strategies
from roboturk.match_limits import GAME_LIMITS, PRACTICE_RANGES, validate_limits


HALF_SECONDS_RANGE = (1, 300)


def session_options(request):
    if not isinstance(request, dict):
        raise ValueError("Choose Practice or Game.")
    timing = {"half_seconds", "periods"}
    if request.get("mode") == "game":
        allowed = timing | {"mode", "team"}
        if request.keys() - allowed or request.get("team") not in ("blue", "yellow"):
            raise ValueError("In Game mode, choose Blue or Yellow as your team.")
    elif request.get("mode") == "practice":
        allowed = timing | {"mode", "strategies", "limits"}
        if request.keys() - allowed or request.get("strategies") is None:
            raise ValueError("In Practice mode, choose a playbook for each team.")
        decode_strategies(request["strategies"])
        if "limits" in request:
            validate_limits(request["limits"])
    else:
        raise ValueError("Choose Practice or Game.")
    if "half_seconds" in request:
        value = request["half_seconds"]
        minimum, maximum = HALF_SECONDS_RANGE
        if type(value) is not int or not minimum <= value <= maximum:
            raise ValueError(
                f"Set the period length to a whole number from {minimum} to {maximum} seconds."
            )
    if "periods" in request:
        value = request["periods"]
        if type(value) is not int or value not in (1, 2):
            raise ValueError("Choose one or two periods.")
    return request


@dataclass
class MatchSession:
    directory: Path
    options: dict
    process: subprocess.Popen
    failed: bool = False

    @property
    def match(self):
        return self.directory / "run"

    @property
    def url(self):
        return f"/matches/{self.directory.name}/"

    @property
    def limit_path(self):
        return self.directory / "motion-limits.json" if self.options["mode"] == "practice" else None


def write_limits(path, values):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(values) + "\n")
    temporary.replace(path)


class MatchRunning(Exception):
    def __init__(self, url):
        self.url = url


class LimitsLocked(Exception):
    pass


class MatchSessions:
    def __init__(self, root, spec, export):
        self.root = root
        self.spec = json.loads(spec.read_text())
        if self.spec.get("mode") != "league_match":
            raise ValueError("The viewer setup requires an official league-match preset.")
        self.timing = dict(half_seconds=self.spec["half_seconds"], periods=self.spec.get("periods", 2))
        self.export = export
        self.sessions = {}
        self.active = None
        self.lock = Lock()

    def start(self, request):
        options = self.timing | session_options(request)
        spec = self.spec | {key: options[key] for key in self.timing}
        playing_seconds = spec["periods"] * spec["half_seconds"]
        # This watchdog also covers stoppages and kickoff setup.
        spec["seconds"] = max(spec.get("seconds", 240), 3 * playing_seconds + 60)
        # Game mode always runs the shipped playbooks.
        spec["team_strategies"] = options.get("strategies")
        spec["team_limits"] = None
        spec["practice_limits_file"] = None
        with self.lock:
            if self.active and self.active.process.poll() is None:
                raise MatchRunning(self.active.url)
            directory = self.root / uuid4().hex[:12]
            directory.mkdir(parents=True)
            if options["mode"] == "practice":
                limit_path = directory / "motion-limits.json"
                write_limits(limit_path, options.get("limits", GAME_LIMITS))
                spec["practice_limits_file"] = str(limit_path)
            spec_file = directory / "spec.json"
            spec_file.write_text(json.dumps(spec, indent=2) + "\n")
            with (directory / "launch.log").open("w") as log:
                process = subprocess.Popen(
                    [
                        sys.executable,
                        str(Path(__file__).with_name("autoref") / "run_placement.py"),
                        str(spec_file),
                        str(directory / "run"),
                        "--live-viewer",
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            session = MatchSession(directory, options, process)
            self.sessions[directory.name] = self.active = session
        Thread(target=self.finish, args=(session,), daemon=True).start()
        return session

    def edit_limits(self, session, changes):
        if not isinstance(changes, dict) or not changes or changes.keys() - PRACTICE_RANGES.keys():
            raise ValueError("Choose a motion limit to change.")
        with self.lock:
            if session.limit_path is None:
                raise LimitsLocked("Game limits are fixed.")
            if session.process.poll() is not None:
                raise LimitsLocked("This practice match is finished.")
            proposed = json.loads(session.limit_path.read_text()) | changes
            write_limits(session.limit_path, validate_limits(proposed))

    def finish(self, session):
        if session.process.wait():
            session.failed = True
            return
        try:
            self.export(session.match, session.match / "viewer.html")
        except (OSError, ValueError) as error:
            with (session.directory / "launch.log").open("a") as log:
                print(f"Replay export failed: {error}", file=log)

    def close(self):
        active = self.active
        if not active or active.process.poll() is not None:
            return
        try:
            os.killpg(active.process.pid, signal.SIGINT)
            active.process.wait(timeout=8)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            os.killpg(active.process.pid, signal.SIGTERM)
