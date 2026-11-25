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
from roboturk.match_limits import team_configs
from roboturk.model import Config


def session_options(request):
    if not isinstance(request, dict):
        raise ValueError("Choose Practice or Game.")
    if request.get("mode") == "game":
        if set(request) != {"mode", "team"} or request["team"] not in ("blue", "yellow"):
            raise ValueError("In Game mode, choose Blue or Yellow as your team.")
    elif request.get("mode") == "practice":
        if set(request) - {"mode", "strategies", "limits"} or request.get("strategies") is None:
            raise ValueError("In Practice mode, choose a playbook for each team.")
        decode_strategies(request["strategies"])
        if "limits" in request:
            if request["limits"] is None:
                raise ValueError("Set motion limits for both teams.")
            team_configs(Config(), officiated=True, practice=request["limits"])
    else:
        raise ValueError("Choose Practice or Game.")
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


class MatchRunning(Exception):
    def __init__(self, url):
        self.url = url


class MatchSessions:
    def __init__(self, root, spec, export):
        self.root = root
        self.spec = json.loads(spec.read_text())
        if self.spec.get("mode") != "league_match":
            raise ValueError("The viewer setup requires an official league-match preset.")
        self.export = export
        self.sessions = {}
        self.active = None
        self.lock = Lock()

    def start(self, request):
        options = session_options(request)
        spec = dict(self.spec)
        # Game mode always runs the shipped playbooks.
        spec["team_strategies"] = options.get("strategies")
        spec["team_limits"] = options.get("limits")
        with self.lock:
            if self.active and self.active.process.poll() is None:
                raise MatchRunning(self.active.url)
            directory = self.root / uuid4().hex[:12]
            directory.mkdir(parents=True)
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
