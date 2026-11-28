"""Export, stream, or run a 5v5 match with Hoca."""

import argparse
from functools import partial
import gzip
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import subprocess
import sys
from threading import Lock, Thread
from urllib.parse import parse_qs, urlsplit

import numpy as np

from controller_versions import MATCH_VERSIONS_FILE, match_releases
from decision_changes import validate_request
from decision_timeline import TRIAL_SECONDS, DecisionTimelines, branch_replay, webots_path
from match_sessions import HALF_SECONDS_RANGE, LimitsLocked, MatchRunning, MatchSessions
from roboturk.five_roles import CFG
from roboturk.five_strategy import PASS_PREFERENCES, PLAYBOOKS
from roboturk.match_limits import GAME_LIMITS, PRACTICE_RANGES, limit_values, team_configs
from roboturk.motion_forecast import open_play_segments, replay_forecasts
from roboturk.role_telemetry import action_events, compact_frame, frame_events, viewer_header
from roboturk.team_forecast import TeamModel, scene_features, team_predictions


TEMPLATES = Path(__file__).with_suffix("")
CHUNK_BYTES = 262_144
TEAM_MODEL = TEMPLATES.parents[1] / "policies/team_forecast_v1/model.npz"


def archive_folder(folder):
    path = Path(folder)
    return path if (path / "roles.json.gz").is_file() else path / "match"


def archive_ready(folder):
    archive = archive_folder(folder)
    return (archive / "roles.json.gz").is_file() and (archive / "positions.npz").is_file()


def read_recording(folder):
    folder = archive_folder(folder)
    with gzip.open(folder / "roles.json.gz", "rt") as source:
        log = json.load(source)
    with np.load(folder / "positions.npz") as records:
        samples = records["states"]
    keys, raw_frames = log["keys"], log["frames"]
    if not raw_frames or samples.shape != (len(raw_frames), 4 + 3 * len(keys)):
        raise ValueError("Positions and roles must contain the same match samples")
    if not np.isfinite(samples).all():
        raise ValueError("Match positions must be finite")
    return log, samples


def live_replay(folder):
    stream = (folder / "telemetry.jsonl")
    if not stream.is_file():
        stream = folder / "match/telemetry.jsonl"
    if not stream.is_file():
        raise ValueError("Wait for the first Webots frame.")
    data = dict(frames=[], positions=[], events=[], intro={"frame": 0, "robot": 1})
    with stream.open() as source:
        for line in source:
            if not line.endswith("\n"):
                break
            entry = json.loads(line)
            if "header" in entry:
                data.update(entry["header"])
            elif "frame" in entry:
                data["frames"].append(entry["frame"])
                data["positions"].append(entry["position"])
                data["events"].extend(entry["events"])
    if not data["frames"]:
        raise ValueError("Wait for the first Webots frame.")
    return data


def live_fork_trace(data):
    frames = [dict(frame, clock=frame["t"], attack_sign=frame["sign"], control=frame["owner"])
              for frame in data["frames"]]
    samples = [position[:2] + (frame["ball_vxy"] or [0, 0]) + position[2:]
               for position, frame in zip(data["positions"], frames, strict=True)]
    return dict(data, frames=frames), samples


def build_data(folder, *, include_forecast=True):
    folder = archive_folder(folder)
    log, samples = read_recording(folder)
    keys, raw_frames = log["keys"], log["frames"]
    dt = log["dt"]
    bundle = None
    if include_forecast:
        forecasts = replay_forecasts(
            samples[:, 4:].reshape(len(samples), len(keys), 3)[..., :2],
            dt,
            open_play_segments(raw_frames, dt),
        )
        bundle = forecasts.viewer_data() if forecasts.valid.any() else None
    if (
        bundle is not None and TEAM_MODEL.is_file() and len(keys) == 10
        and any(frame.get("allocation") for frame in raw_frames)
    ):
        scene = scene_features(log, samples)
        paths = team_predictions(log, samples, scene, TeamModel.load(TEAM_MODEL))
        bundle["team_paths"] = {
            team: {int(tick): row.round(4).ravel().tolist()
                   for tick, row in zip(scene.origins, trajectory, strict=True)}
            for team, trajectory in paths.items()
        }
    frames, events = [], []
    previous = None
    for index, frame in enumerate(raw_frames):
        if len(frame["roles"]) != len(keys):
            raise ValueError(f"Incomplete roster at sample {index}")
        details = frame.get("allocation")
        if details is not None and len(details) != len(keys):
            raise ValueError(f"Incomplete role audit at sample {index}")
        frames.append(
            compact_frame(frame, index, log["dt"], log.get("sign", 1), ball_vxy=samples[index, 2:4])
        )
        events.extend(frame_events(previous, frame, index))
        previous = frame
    events.extend(
        entry for entry in action_events(log["events"], keys) if 0 <= entry[0] < len(frames)
    )
    intro = {"frame": 0, "robot": 1}
    for event in log["events"]:
        if event["event"] in ("pass", "restart_pass") and event["tick"] + 1 < len(frames):
            intro = {"frame": event["tick"] + 1, "robot": event["receiver"]}
            break
    limits = log.get("limits")
    if limits is None:
        configs = team_configs(
            CFG, officiated=any(frame.get("ssl_command") for frame in raw_frames)
        )
        limits = {team: limit_values(config) for team, config in configs.items()}
    config_path = folder / "config.json"
    team_strategies = None
    if config_path.is_file():
        team_strategies = json.loads(config_path.read_text()).get("team_strategies")
    recording = dict(
        viewer_header(keys, log["seed"], CFG, limits),
        dt=log["dt"],
        frames=frames,
        positions=np.round(np.concatenate((samples[:, :2], samples[:, 4:]), axis=1), 4).tolist(),
        events=sorted(events, key=lambda event: event[0]),
        intro=intro,
        audited=any(frame["allocation"] is not None for frame in frames),
        team_strategies=team_strategies,
        forecast=bundle,
    )
    for directory in (folder, folder.parent) if folder.name == "match" else (folder,):
        versions = directory / MATCH_VERSIONS_FILE
        if versions.is_file():
            recording["controller_versions"] = json.loads(versions.read_text())
            break
    return recording


def page(match, template="index", script="viewer"):
    data = json.dumps(
        dict(match, limit_ranges=PRACTICE_RANGES), separators=(",", ":"), allow_nan=False
    ).replace("<", "\\u003c")
    html = (TEMPLATES / f"{template}.html").read_text()
    html = html.replace("/* VIEWER_STYLES */", (TEMPLATES / "viewer.css").read_text())
    html = html.replace("/* LIMIT_CONTROLS */", (TEMPLATES / "limits.js").read_text())
    html = html.replace("/* MATCH_REVIEW */", (TEMPLATES / "review.js").read_text())
    html = html.replace("/* TIMELINE_CONTROLS */", (TEMPLATES / "timeline.js").read_text())
    html = html.replace("/* VIEWER_SCRIPT */", (TEMPLATES / f"{script}.js").read_text())
    return html.replace("<!-- MATCH_DATA -->", data)


def export(folder, output):
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(page(build_data(folder)))
    print(output, flush=True)


class ViewerRequest(BaseHTTPRequestHandler):
    def __init__(self, *args, folder, html, **kwargs):
        self.folder = folder
        self.html = html
        super().__init__(*args, **kwargs)

    def log_request(self, code="-", size="-"):
        pass  # The browser polls several times a second.

    def respond(self, content, content_type, offset=None, *, status=200, failed=False, continuation=None, handoff=False):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        if offset is not None:
            self.send_header("X-Telemetry-Offset", str(offset))
        if failed:
            self.send_header("X-Match-Failed", "1")
        if continuation:
            self.send_header("X-Hoca-Continuation", continuation)
        if handoff:
            self.send_header("X-Hoca-Handoff", "1")
        self.end_headers()
        self.wfile.write(content)

    def do_GET(self):
        url = urlsplit(self.path)
        if url.path == "/":
            html = page(build_data(self.folder)).encode() if archive_ready(self.folder) else self.html
            self.respond(html, "text/html; charset=utf-8")
            return
        if url.path != "/telemetry":
            self.send_error(404)
            return
        self.telemetry(self.folder, url.query)

    def telemetry(self, folder, query, failed=False, continuation=None, handoff=False):
        try:
            offset = int(parse_qs(query).get("offset", ["0"])[0])
            if not 0 <= offset < 2**63:
                raise ValueError
        except ValueError:
            self.send_error(400, "Invalid telemetry offset")
            return

        stream = folder / "telemetry.jsonl"
        if not stream.is_file():
            stream = folder / "match/telemetry.jsonl"
        try:
            with stream.open("rb") as source:
                source.seek(offset)
                chunk = source.read(CHUNK_BYTES)
        except FileNotFoundError:
            chunk = b""
        # Webots may be writing the last record as this request arrives.
        chunk = chunk[: chunk.rfind(b"\n") + 1]
        self.respond(chunk, "application/x-ndjson", offset + len(chunk), failed=failed and not handoff,
                     continuation=continuation, handoff=handoff)


class SessionRequest(ViewerRequest):
    def __init__(self, *args, sessions, html, **kwargs):
        self.sessions = sessions
        super().__init__(*args, folder=None, html=html, **kwargs)

    def do_GET(self):
        url = urlsplit(self.path)
        if url.path == "/":
            self.respond(self.html, "text/html; charset=utf-8")
            return
        parts = url.path.split("/")
        if len(parts) == 4 and parts[1] in ("controller-replays", "matches") and parts[3] == "timeline":
            record = self.sessions.timelines.read(url.path.removesuffix("timeline"))
            self.json_response({"timeline": record})
            return
        if len(parts) == 4 and parts[1] in ("controller-replays", "matches") and parts[3] == "timeline-stream":
            timelines = self.sessions.timelines
            record = timelines.read(url.path.removesuffix("timeline-stream"))
            if record and record.get("promoted"):
                self.telemetry(timelines.trial_folder(record), url.query, record["status"] == "failed")
            else:
                self.send_error(404)
            return
        if len(parts) == 4 and parts[1] == "controller-replays" and not parts[3]:
            moment = self.sessions.catalog["moments"].get(parts[2])
            if moment is None:
                self.send_error(404)
                return
            folder = TEMPLATES.parents[1] / moment["archive"]
            if not archive_ready(folder):
                self.send_error(404, "Archived replay not installed")
                return
            self.recording(folder, url, replay_moment=parts[2],
                           intro=dict(frame=moment["frame"], robot=moment["robot"]))
            return
        if len(parts) != 4 or parts[1] != "matches":
            self.send_error(404)
            return
        match_id = parts[2]
        session = self.sessions.sessions.get(match_id)
        source = f"/matches/{match_id}/"
        record = self.sessions.timelines.read(source)
        frozen = self.sessions.timelines.directory(source) / "parent.json"
        if session is None:
            if parts[3] or len(match_id) != 12 or any(c not in "0123456789abcdef" for c in match_id):
                self.send_error(404)
                return
            archived = self.sessions.root / match_id / "run"
            if not (archived / "viewer.html").is_file() and not frozen.is_file():
                self.send_error(404)
                return
            self.recording(archived, url)
        elif parts[3] == "":
            self.recording(session.match, url, live_session=session, session=session.options)
        elif parts[3] == "telemetry":
            failed = session.failed
            record = self.sessions.timelines.read(source)
            continuation = source + "?timeline=branch" if record and record.get("promoted") else None
            handoff = record and record["status"] == "switching"
            self.telemetry(session.match, url.query, failed and not continuation, continuation, handoff)
        else:
            self.send_error(404)

    def json_response(self, value, status=200):
        self.respond(json.dumps(value).encode(), "application/json", status=status)

    def recording(self, folder, url, live_session=None, **metadata):
        timelines = self.sessions.timelines
        record = timelines.read(url.path)
        on_branch = record is not None and parse_qs(url.query).get("timeline") == ["branch"]
        frozen = timelines.directory(url.path) / "parent.json"
        if archive_ready(folder):
            parent = build_data(folder, include_forecast=not on_branch)
        elif frozen.is_file() and (not live_session or live_session.process.poll() is not None):
            parent = json.loads(frozen.read_text())
        elif live_session and record:
            parent = dict(live_replay(folder), controller_versions=live_session.controller_versions)
        else:
            parent = {}
        data = dict(parent, setup_url="/", **metadata)
        if record:
            data["decision_timeline"] = dict(
                record, parent_end=data["frames"][-1]["t"], parent_last=len(data["frames"]) - 1,
                preview_seconds=TRIAL_SECONDS,
            )
            if on_branch:
                run_folder = timelines.trial_folder(record) if record["run"] else None
                streaming = record["status"] == "continuing"
                preview = run_folder / "preview" if run_folder and archive_ready(run_folder / "preview") else run_folder
                archive = run_folder if record["status"] == "finished" else preview
                trial = (build_data(archive, include_forecast=False)
                         if archive and archive_ready(archive) and not streaming else None)
                data = branch_replay(data, record, trial, streaming=streaming)
                if streaming:
                    data.update(live=True, telemetry_url="timeline-stream")
        if live_session and not on_branch and not frozen.is_file() and not archive_ready(folder):
            for field in ("frames", "positions", "events", "keys"):
                data.pop(field, None)
            data.update(live=True, controller_versions=live_session.controller_versions,
                        edit_limits=live_session.limit_path is not None and live_session.process.poll() is None)
        if not data.get("controller_versions"):
            data["legacy_controllers"] = match_releases(self.sessions.catalog, None, data.get("team_strategies"))
        self.respond(page(data).encode(), "text/html; charset=utf-8")

    def recording_folder(self, source):
        parts = source.split("/")
        folder = None
        if len(parts) == 4 and not parts[0] and not parts[3]:
            if parts[1] == "controller-replays":
                moment = self.sessions.catalog["moments"].get(parts[2])
                if moment:
                    folder = TEMPLATES.parents[1] / moment["archive"]
            elif parts[1] == "matches" and len(parts[2]) == 12 and all(c in "0123456789abcdef" for c in parts[2]):
                session = self.sessions.sessions.get(parts[2])
                if not session or not session.failed:
                    folder = self.sessions.root / parts[2] / "run"
        if folder is None or not (archive_ready(folder) or (archive_folder(folder) / "telemetry.jsonl").is_file()):
            raise ValueError("Open a Hoca match with recorded frames to branch the timeline.")
        return archive_folder(folder)

    def stop_parent(self, source, active):
        if not active or active.url != source or active.process.poll() is not None:
            return
        live_replay(active.match)
        self.sessions.stop(active)
        parent = dict(live_replay(active.match), controller_versions=active.controller_versions)
        frozen = self.sessions.timelines.directory(source) / "parent.json"
        pending = frozen.with_suffix(".tmp")
        pending.write_text(json.dumps(parent, allow_nan=False))
        pending.replace(frozen)

    def change_timeline(self, source, request):
        with self.sessions.launch_lock:
            active = self.sessions.active
            if active and active.process.poll() is None and active.url != source:
                raise MatchRunning(active.url)
            timelines = self.sessions.timelines
            if request == {"action": "continue"}:
                return timelines.promote(source, lambda: self.stop_parent(source, active))
            if request != {}:
                raise ValueError("Choose to run the trial or continue the merged branch.")
            folder = self.recording_folder(source)
            log, samples = (read_recording(folder) if archive_ready(folder)
                            else live_fork_trace(live_replay(folder)))
            config = folder / "config.json"
            fork = timelines.read(source)
            if fork is None:
                raise ValueError("Create a candidate branch first.")
            elapsed = log["frames"][-1]["clock"]
            if active and active.url == source and active.process.poll() is None:
                elapsed = max(elapsed, active.options["half_seconds"] * active.options["periods"])
            return timelines.run(
                source, log, samples, json.loads(config.read_text()) if config.is_file() else {},
                round(elapsed - fork["fork_time"]),
            )

    def do_POST(self):
        session = None
        decision_change = self.path == "/decision-changes"
        run_trial = self.path.endswith("/timeline")
        if self.path != "/matches" and not decision_change and not run_trial:
            parts = self.path.split("/")
            if len(parts) == 4 and parts[1] == "matches" and parts[3] == "limits":
                session = self.sessions.sessions.get(parts[2])
            if session is None:
                self.send_error(404)
                return
        origin = f"http://127.0.0.1:{self.server.server_port}"
        if (
            self.headers.get("Origin") != origin
            or self.headers.get_content_type() != "application/json"
        ):
            self.json_response({"error": "Open the local Hoca address to change a match."}, 403)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            maximum = 16 * 1024 * 1024 if decision_change else 4096
            if not 0 < length <= maximum:
                raise ValueError("The request is empty or exceeds the local upload size limit.")
            request = json.loads(self.rfile.read(length))
            if decision_change:
                validate_request(request)
                clip_source = request["evidence"]["source"]
                if clip_source.get("replay_moment"):
                    source = f"/controller-replays/{clip_source['replay_moment']}/"
                else:
                    source = f"/matches/{clip_source.get('match_id')}/"
                folder = self.recording_folder(source)
                log, _ = read_recording(folder) if archive_ready(folder) else live_fork_trace(live_replay(folder))
                record, created = self.sessions.timelines.create(source, request, log)
                self.json_response(dict(record, created=created), 201 if created else 200)
                return
            if run_trial:
                source = self.path.removesuffix("timeline")
                self.json_response(self.change_timeline(source, request), 202)
                return
            if session is not None:
                self.sessions.edit_limits(session, request)
                self.json_response({"saved": True})
                return
            with self.sessions.launch_lock:
                if self.sessions.timelines.running:
                    current = self.sessions.timelines.read(self.sessions.timelines.running)
                    if current["status"] != "ready":
                        raise LimitsLocked("Wait for the active Webots branch to finish.")
                session = self.sessions.start(request)
        except MatchRunning as error:
            self.json_response({"error": "A match is already running.", "url": error.url}, 409)
        except LimitsLocked as error:
            self.json_response({"error": str(error)}, 409)
        except ValueError as error:
            self.json_response({"error": str(error)}, 400)
        except OSError:
            error = "Could not create the local candidate." if decision_change else "Could not update the local Webots match."
            self.json_response({"error": error}, 500)
        else:
            self.json_response({"url": session.url}, 201)


def viewer_server(folder, port):
    handler = partial(ViewerRequest, folder=folder, html=page({"live": True}).encode())
    return ThreadingHTTPServer(("127.0.0.1", port), handler)


def serve_setup(folder, port, spec):
    sessions = MatchSessions(folder.resolve(), spec, export)
    sessions.launch_lock = Lock()
    sessions.timelines = DecisionTimelines(sessions.root, webots_path(sessions.spec))
    setup = dict(
        strategies={key: [book.name, book.description] for key, book in PLAYBOOKS.items()},
        controller_catalog=sessions.catalog,
        passing=PASS_PREFERENCES,
        limit_defaults=GAME_LIMITS,
        timing=sessions.timing,
        half_seconds_range=HALF_SECONDS_RANGE,
    )
    handler = partial(
        SessionRequest, sessions=sessions, html=page(setup, "setup", "setup").encode()
    )
    try:
        with ThreadingHTTPServer(("127.0.0.1", port), handler) as server:
            print(f"Hoca setup http://127.0.0.1:{server.server_port}/", flush=True)
            server.serve_forever()
    finally:
        try:
            sessions.timelines.close()
        finally:
            sessions.close()


def serve(folder, port, spec=None):
    folder = folder.resolve()
    with viewer_server(folder, port) as server:
        print(f"Hoca http://127.0.0.1:{server.server_port}/", flush=True)
        if spec is not None:
            server_thread = Thread(target=server.serve_forever, daemon=True)
            server_thread.start()
            try:
                subprocess.run(
                    [
                        sys.executable,
                        str(TEMPLATES.parent / "autoref/run_placement.py"),
                        str(spec.resolve()),
                        str(folder),
                        "--live-viewer",
                    ],
                    check=True,
                )
                export(folder, folder / "viewer.html")
                print("Match finished. Press Ctrl-C here to close Hoca.", flush=True)
                server_thread.join()
            finally:
                server.shutdown()
        else:
            server.serve_forever()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("match", type=Path, help="Match or match-run directory")
    parser.add_argument("output", type=Path, nargs="?", help="Standalone HTML file to write")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--serve", action="store_true", help="Serve a directory with telemetry")
    modes.add_argument(
        "--play", type=Path, metavar="SPEC", help="Run an official match and serve it"
    )
    modes.add_argument(
        "--setup", type=Path, metavar="SPEC", help="Choose Practice or Game in the browser"
    )
    parser.add_argument("--port", type=int, default=8765, help="Loopback port for Hoca")
    args = parser.parse_args()
    if args.serve or args.play or args.setup:
        if args.output or (args.play and args.match.exists()):
            parser.error("Use a fresh match directory with --play and omit the HTML output path")
        try:
            if args.setup:
                serve_setup(args.match, args.port, args.setup)
            else:
                serve(args.match, args.port, args.play)
        except KeyboardInterrupt:
            pass
    elif args.output:
        export(args.match, args.output)
    else:
        parser.error("Provide an output HTML path, --serve, or --play")


if __name__ == "__main__":
    main()
