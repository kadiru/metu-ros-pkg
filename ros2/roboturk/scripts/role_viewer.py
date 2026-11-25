"""Export, stream, or run a 5v5 match with the role viewer."""

import argparse
from functools import partial
import gzip
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import subprocess
import sys
from threading import Thread
from urllib.parse import parse_qs, urlsplit

import numpy as np

from roboturk.five_roles import CFG
from roboturk.role_telemetry import action_events, compact_frame, frame_events, viewer_header


TEMPLATES = Path(__file__).with_suffix("")
CHUNK_BYTES = 262_144


def archive_folder(folder):
    path = Path(folder)
    return path if (path / "roles.json.gz").is_file() else path / "match"


def build_data(folder):
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
    frames, events = [], []
    previous = None
    for index, frame in enumerate(raw_frames):
        if len(frame["roles"]) != len(keys):
            raise ValueError(f"Incomplete roster at sample {index}")
        details = frame.get("allocation")
        if details is not None and len(details) != len(keys):
            raise ValueError(f"Incomplete role audit at sample {index}")
        frames.append(compact_frame(frame, index, log["dt"], log.get("sign", 1)))
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
    return dict(
        viewer_header(keys, log["seed"], CFG),
        dt=log["dt"],
        frames=frames,
        positions=np.round(np.concatenate((samples[:, :2], samples[:, 4:]), axis=1), 4).tolist(),
        events=sorted(events, key=lambda event: event[0]),
        intro=intro,
        audited=any(frame["allocation"] is not None for frame in frames),
    )


def page(match):
    data = json.dumps(match, separators=(",", ":"), allow_nan=False).replace("<", "\\u003c")
    html = (TEMPLATES / "index.html").read_text()
    html = html.replace("/* VIEWER_STYLES */", (TEMPLATES / "viewer.css").read_text())
    html = html.replace("/* VIEWER_SCRIPT */", (TEMPLATES / "viewer.js").read_text())
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

    def respond(self, content, content_type, offset=None):
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        if offset is not None:
            self.send_header("X-Telemetry-Offset", str(offset))
        self.end_headers()
        self.wfile.write(content)

    def do_GET(self):
        url = urlsplit(self.path)
        if url.path == "/":
            self.respond(self.html, "text/html; charset=utf-8")
            return
        if url.path != "/telemetry":
            self.send_error(404)
            return
        try:
            offset = int(parse_qs(url.query).get("offset", ["0"])[0])
            if not 0 <= offset < 2**63:
                raise ValueError
        except ValueError:
            self.send_error(400, "Invalid telemetry offset")
            return

        stream = self.folder / "telemetry.jsonl"
        if not stream.is_file():
            stream = self.folder / "match/telemetry.jsonl"
        try:
            with stream.open("rb") as source:
                source.seek(offset)
                chunk = source.read(CHUNK_BYTES)
        except FileNotFoundError:
            chunk = b""
        # Webots may be writing the last record as this request arrives.
        chunk = chunk[: chunk.rfind(b"\n") + 1]
        self.respond(chunk, "application/x-ndjson", offset + len(chunk))


def viewer_server(folder, port):
    handler = partial(ViewerRequest, folder=folder, html=page({"live": True}).encode())
    return ThreadingHTTPServer(("127.0.0.1", port), handler)


def serve(folder, port, spec=None):
    folder = folder.resolve()
    with viewer_server(folder, port) as server:
        print(f"Match viewer http://127.0.0.1:{server.server_port}/", flush=True)
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
                print("Match finished. Press Ctrl-C here to close the live viewer.", flush=True)
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
    parser.add_argument("--port", type=int, default=8765, help="Loopback port for the live viewer")
    args = parser.parse_args()
    if args.serve or args.play:
        if args.output or (args.play and args.match.exists()):
            parser.error("Use a fresh match directory with --play and omit the HTML output path")
        try:
            serve(args.match, args.port, args.play)
        except KeyboardInterrupt:
            pass
    elif args.output:
        export(args.match, args.output)
    else:
        parser.error("Provide an output HTML path, --serve, or --play")


if __name__ == "__main__":
    main()
