"""Export a recorded 5v5 match as a standalone, offline role viewer."""

import argparse
import gzip
import json
from pathlib import Path

import numpy as np

TEMPLATES = Path(__file__).with_suffix("")
STAGES = {
    "NORMAL_FIRST_HALF_PRE": "First kickoff",
    "NORMAL_FIRST_HALF": "First half",
    "NORMAL_HALF_TIME": "Halftime",
    "NORMAL_SECOND_HALF_PRE": "Second kickoff",
    "NORMAL_SECOND_HALF": "Second half",
    "POST_GAME": "Full time",
}


def robot_name(keys, index):
    team, number = keys[index]
    return f"{team.title()} {number}"


def match_events(frames, events, keys):
    timeline = []
    for index, (previous, current) in enumerate(zip(frames, frames[1:]), 1):
        if previous["score"] != current["score"]:
            blue, yellow = current["score"]
            team = "Blue" if blue > previous["score"][0] else "Yellow"
            timeline.append([index, f"{team} goal · {blue}–{yellow}"])
        stage = current.get("stage")
        if stage != previous.get("stage") and stage in STAGES:
            timeline.append([index, STAGES[stage]])
    for event in events:
        tick, kind = event["tick"], event["event"]
        if not 0 <= tick < len(frames):
            continue
        if kind in ("pass", "restart_pass"):
            actor = robot_name(keys, event["actor"])
            target = robot_name(keys, event["receiver"])
            timeline.append([tick, f"{actor} passes to {target}"])
        elif kind in ("kickoff_kick", "shot"):
            action = "kicks off" if kind == "kickoff_kick" else "shoots"
            timeline.append([tick, f"{robot_name(keys, event['actor'])} {action}"])
    return sorted(timeline, key=lambda event: event[0])


def build_data(folder):
    folder = Path(folder)
    if not (folder / "roles.json.gz").exists():
        folder = folder / "match"
    with gzip.open(folder / "roles.json.gz", "rt") as source:
        log = json.load(source)
    with np.load(folder / "positions.npz") as records:
        samples = records["states"]
    keys, raw_frames = log["keys"], log["frames"]
    if not raw_frames or samples.shape != (len(raw_frames), 4 + 3 * len(keys)):
        raise ValueError("Positions and roles must contain the same match samples")
    if not np.isfinite(samples).all():
        raise ValueError("Match positions must be finite")
    frames = []
    for index, frame in enumerate(raw_frames):
        if len(frame["roles"]) != len(keys):
            raise ValueError(f"Incomplete roster at sample {index}")
        frame_details = frame.get("allocation")
        if frame_details is not None and len(frame_details) != len(keys):
            raise ValueError(f"Incomplete role audit at sample {index}")
        motions = frame.get("motion")
        frames.append(
            dict(
                t=frame.get("clock", round(index * log["dt"], 4)),
                score=frame["score"],
                roles=frame["roles"],
                phases=frame["phases"],
                stage=STAGES.get(frame.get("stage"), ""),
                left=frame.get("stage_seconds"),
                command=frame.get("ssl_command", ""),
                mode=frame.get("mode", "running"),
                owner=frame.get("control"),
                sign=frame.get("attack_sign", log.get("sign", 1)),
                restricted=frame.get("restricted_to", ""),
                allocation=frame_details,
                motion=[m.get("mode") if m else None for m in motions] if motions else None,
            )
        )
    intro = {"frame": 0, "robot": 1}
    for event in log["events"]:
        if event["event"] not in ("pass", "restart_pass"):
            continue
        index = event["tick"] + 1
        if index < len(frames):
            intro = {"frame": index, "robot": event["receiver"]}
            break
    return dict(
        keys=keys,
        dt=log["dt"],
        seed=log["seed"],
        frames=frames,
        positions=np.round(np.concatenate((samples[:, :2], samples[:, 4:]), axis=1), 4).tolist(),
        events=match_events(raw_frames, log["events"], keys),
        intro=intro,
        audited=any(frame["allocation"] is not None for frame in frames),
    )


def export(folder, output):
    data = json.dumps(build_data(folder), separators=(",", ":"), allow_nan=False).replace(
        "<", "\\u003c"
    )
    # Escape the end of a script block even when given traces from other tools.
    html = (TEMPLATES / "index.html").read_text()
    html = html.replace("/* VIEWER_STYLES */", (TEMPLATES / "viewer.css").read_text())
    html = html.replace("/* VIEWER_SCRIPT */", (TEMPLATES / "viewer.js").read_text())
    html = html.replace("<!-- MATCH_DATA -->", data)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("match", type=Path, help="Match or match-run directory")
    parser.add_argument("output", type=Path, help="Standalone HTML file to write")
    args = parser.parse_args()
    export(args.match, args.output)
    print(args.output)


if __name__ == "__main__":
    main()
