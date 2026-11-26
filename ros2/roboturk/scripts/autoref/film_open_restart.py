"""Verify the free kick in a continuous 5v5 match and caption its Webots frames."""

import argparse
import json
import math
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw
from film import font
from verify import load_events, load_log


def verify(run, include_open_play_fouls=False):
    log = load_log(run / "match")
    frames = log["frames"]
    with np.load(run / "match/positions.npz") as archive:
        s = archive["states"]
    records = [
        json.loads(line)["change"]
        for line in (run / "gc/config/state-store.json.stream").read_text().splitlines()
    ]
    official = [c["addGameEventChange"]["gameEvent"] for c in records if "addGameEventChange" in c]
    observed = load_events(run / "tigers.log")
    boundary = next(e for e in official if e["type"] == "BALL_LEFT_FIELD_TOUCH_LINE")
    accepted = next(e for e in official if e["type"] == "PLACEMENT_SUCCEEDED")
    assert all("TIGERs AutoRef" in e["origin"] for e in (boundary, accepted))
    own = accepted["placementSucceeded"]["byTeam"].lower()
    assert boundary["ballLeftFieldTouchLine"]["byTeam"].lower() != own
    assert any(
        e.get("placementSucceeded") == accepted["placementSucceeded"] for _, _, e in observed
    )
    first = next(
        i for i, f in enumerate(frames) if f["ssl_command"] == "BALL_PLACEMENT_" + own.upper()
    )
    (call,) = [
        (t, e["ballLeftFieldTouchLine"])
        for t, k, e in observed
        if k == "ball_left_field_touch_line"
    ]
    boundary_time, last_touch = call
    designated = boundary["ballLeftFieldTouchLine"]
    assert (last_touch["byTeam"], last_touch["byBot"]) == (
        designated["byTeam"],
        designated["byBot"],
    )
    assert abs(last_touch["location"]["x"] - designated["location"]["x"]) < 0.001
    assert last_touch["location"]["y"] * designated["location"]["y"] > 0
    fouls = []
    for t, kind, event in observed:
        if kind in ("bot_crash_drawn", "bot_crash_unique", "bot_interfered_placement"):
            assert include_open_play_fouls and t < boundary_time, (kind, t)
            fouls.append(
                dict(
                    seconds=round(t - frames[0]["vision_time"], 2),
                    type=kind,
                    official=any(
                        event["type"] == e["type"] and all(e.get(k) == v for k, v in event.items())
                        for e in official
                    ),
                )
            )
    direct = next(
        i for i, f in enumerate(frames) if f["ssl_command"] == "DIRECT_FREE_" + own.upper()
    )
    (passed,) = [e for e in log["events"] if e["event"] == "restart_pass"]
    (got,) = [e for e in log["events"] if e["event"] == "restart_received"]
    assert first < direct <= passed["tick"] < got["tick"]
    assert log["keys"][passed["actor"]][0] == own == log["keys"][got["actor"]][0]
    assert passed["receiver"] == got["actor"] and passed["actor"] != got["actor"]
    assert (np.abs(s[:first, 1]) > 3.0215).any() and len(frames) == len(s)
    receiver = got["actor"]
    for tick in range(got["tick"] - 4, got["tick"] + 1):
        row = s[tick]
        x, y, yaw = row[4 + 3 * receiver : 7 + 3 * receiver]
        dx, dy = row[0] - x, row[1] - y
        assert abs(dx * math.cos(yaw) + dy * math.sin(yaw) - 0.115) < 0.035
        assert (
            abs(-dx * math.sin(yaw) + dy * math.cos(yaw)) < 0.045 and math.hypot(*row[2:4]) < 0.35
        )
    distance = math.dist(s[passed["tick"], :2], s[got["tick"], :2])
    assert distance > 1 and max(f["restart"] for f in frames) == 0
    return (
        dict(
            seed=log["seed"],
            team=own,
            prior_open_play_fouls=fouls,
            restart_crashes=0,
            restart_interference=0,
            placer=log["keys"][passed["actor"]][1],
            receiver=log["keys"][receiver][1],
            seconds=log["seconds"],
            kick_tick=passed["tick"],
            receive_tick=got["tick"],
            distance_m=distance,
            placement_tick=first,
            direct_tick=direct,
            controller_events=official,
            detector_events=[p for _, _, p in observed],
        ),
        log,
        s,
    )


def render(run, dest, include_open_play_fouls=False):
    audit, log, states = verify(run, include_open_play_fouls)
    dest.mkdir(parents=True, exist_ok=True)
    (dest / "open-match.json").write_text(json.dumps(audit, indent=2) + "\n")
    files = sorted((run / "match/frames").glob("*.jpg"))
    assert len(files) == (len(states) + 1) // 2
    sample = np.asarray(Image.open(files[0])).astype(float)
    h, w = sample.shape[:2]
    r, g, b = sample[h // 3].T
    floor = np.flatnonzero((g > 1.5 * r) & (g > 1.35 * b))
    px = (floor[-1] - floor[0] + 1) / log.get("carpet_length", 10)
    cx, cy = (w - 1) / 2, (h - 1) / 2
    left, top = cx - 5.05 * px, cy - 3.55 * px
    box = (round(left), round(top), round(cx + 5.05 * px), round(cy + 3.55 * px))
    width, height = 850, 598
    movie = dest / "open-match.mp4"
    cmd = [
        "ffmpeg",
        "-v",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{width}x{height + 88}",
        "-r",
        "25/2",
        "-i",
        "-",
        "-c:v",
        "libx264",
        "-crf",
        "18",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(movie),
    ]
    with subprocess.Popen(cmd, stdin=subprocess.PIPE) as proc:
        for i, p in enumerate(files):
            tick = i * 2
            f = log["frames"][tick]
            frame = Image.new("RGB", (width, height + 88), "#10212b")
            native = Image.open(p).convert("RGB").crop(box)
            frame.paste(native.resize((width, height), Image.Resampling.LANCZOS), (0, 43))
            d = ImageDraw.Draw(frame)

            def point(x, y, z=0):
                scale = 16.5 / (16.5 - z)
                return (
                    (cx + scale * x * px - left) * width / (box[2] - box[0]),
                    (cy - scale * y * px - top) * height / (box[3] - box[1]) + 43,
                )

            for key, (x, y, _) in zip(log["keys"], states[tick, 4:].reshape(-1, 3), strict=True):
                u, v = point(x, y, 0.15)
                d.text(
                    (u + 10, v - 13),
                    str(key[1]),
                    font=font(14),
                    fill="#b1daff" if key[0] == "blue" else "#ffe394",
                )
            x, y = point(*states[tick, :2], 0.022)
            d.ellipse((x - 5, y - 5, x + 5, y + 5), outline="#ffbc60", width=2)
            if f.get("target") and tick < audit["direct_tick"]:
                tx, ty = point(*f["target"])
                d.line((tx - 7, ty, tx + 7, ty), fill="#7ef0da", width=2)
                d.line((tx, ty - 7, tx, ty + 7), fill="#7ef0da", width=2)
            d.text((12, 8), "5v5   Open play", font=font(22), fill="white")
            d.text(
                (width - 12, 10),
                f"{f['clock']:.1f} s    {f['ssl_command'].replace('_', ' ')}",
                font=font(17),
                anchor="ra",
                fill="#c3ddf3",
            )
            label = "Both teams choose passes and roles in open play."
            for foul in audit["prior_open_play_fouls"]:
                if foul["seconds"] <= f["clock"] < foul["seconds"] + 2.0:
                    label = "TIGERs reports an open-play collision."
            if tick >= audit["direct_tick"]:
                label = f"{audit['team'].title()} {audit['placer']} takes the free kick to teammate {audit['receiver']}."
            elif f.get("placement_phase"):
                phase = f["placement_phase"]
                label = {
                    "capture": "The placer is collecting the ball.",
                    "carry": "The placer dribbles to the referee’s mark.",
                    "retreat": "The placer backs away. TIGERs checks the restart.",
                }[phase]
            elif f["ssl_command"] == "STOP":
                label = "The Game Controller stops play."
            d.text((12, height + 54), label, font=font(18), fill="#ffedba")
            proc.stdin.write(frame.tobytes())
        proc.stdin.close()
        if proc.wait():
            raise RuntimeError("Could not encode the Webots match")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument(
        "--include-open-play-fouls",
        action="store_true",
        help="Show earlier fouls in the film and still require a clean restart from the touchline call.",
    )
    args = p.parse_args()
    render(args.run, args.output, args.include_open_play_fouls)
