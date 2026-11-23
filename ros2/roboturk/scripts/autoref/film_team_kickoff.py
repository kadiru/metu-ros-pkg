"""Show matched native kickoff footage and an opponent's camera-based motion estimate."""

import argparse
import json
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw

from film_league import FONTS
from film_open_motion import BG, BLUE, WHITE, YELLOW, encode, native_camera, pitch
from roboturk.five_roles import CFG
from roboturk.team_kickoff import STEERING
from run_league import TEAM_ARMS


def save_clip(path, frames):
    encode(path, frames, 12.5)
    subprocess.run(
        [
            "ffmpeg",
            "-v",
            "error",
            "-y",
            "-i",
            str(path),
            "-vf",
            "split[a][b];[a]palettegen[p];[b][p]paletteuse=dither=bayer",
            "-loop",
            "0",
            str(path.with_suffix(".gif")),
        ],
        check=True,
    )


def positions(rec):
    return rec[1][:, 4:].reshape(-1, 10, 3)[:, :, :2]


def comparison(recordings):
    duration = max((len(r[1]) - 1) * CFG.dt for r in recordings)
    width, height = 448, 280
    for sec in np.arange(0, duration + 1.1, 0.16):
        im = Image.new("RGB", (2 * width + 4, 400), BG)
        d = ImageDraw.Draw(im)
        d.text((12, 9), "Ten robots change ends", font=FONTS(25), fill=WHITE)
        d.text(
            (im.width - 14, 16),
            f"Same starting positions   ·   {sec:.1f} s   ·   2×",
            anchor="ra",
            font=FONTS(16),
            fill=WHITE,
        )
        for i, rec in enumerate(recordings):
            end = len(rec[1]) - 1
            t = min(round(sec / CFG.dt), end) // 2 * 2
            x = i * (width + 4)
            xy = positions(rec)
            speed = np.linalg.norm(xy[t] - xy[max(0, t - 1)], axis=1) / CFG.dt
            moving = np.flatnonzero(speed > 0.2)
            board = pitch(rec, t, (0, 0), 10, (width, height), ())
            ink = ImageDraw.Draw(board)
            for k in moving:
                scale = width / 10 * 16.5 / (16.5 - 0.15)
                pts = [
                    (width / 2 + u * scale, height / 2 - v * scale)
                    for u, v in xy[max(0, t - 12) : t + 1, k]
                ]
                if len(pts) > 1:
                    ink.line(pts, fill=BLUE if k < 5 else YELLOW, width=2)
            im.paste(board, (x, 77))
            d.text(
                (x + 12, 45),
                ("Shared routes", "Each team plans its own routes")[i],
                font=FONTS(20),
                fill=WHITE,
            )
            arrived = sec >= end * CFG.dt
            msg = (
                f"Ready at {end * CFG.dt:.2f} s   ·   frame held"
                if arrived
                else f"{len(moving)} moving"
            )
            d.text((x + 12, 368), msg, font=FONTS(18), fill="#9fe4bf" if arrived else WHITE)
        yield im


def encounter(rec):
    xy = positions(rec)
    differences = np.diff(xy, axis=0, prepend=xy[:1]) / CFG.dt
    speed = np.linalg.norm(differences, axis=2)
    distance = np.linalg.norm(xy[:, :5, None, :] - xy[:, None, 5:, :], axis=3)
    moving = (speed[:, :5, None] > 0.3) & (speed[:, None, 5:] > 0.3)
    # Film a full encounter, with enough approach and separation visible on camera.
    window = 10
    encounters = np.zeros_like(moving)
    if len(xy) > 2 * window:
        middle = distance[window:-window]
        encounters[window:-window] = (
            moving[window:-window]
            & (middle <= distance[window - 1 : -window - 1])
            & (middle <= distance[window + 1 : -window + 1])
            & (distance[: -2 * window] > middle + 0.06)
            & (distance[2 * window :] > middle + 0.06)
        )
    candidates = encounters if encounters.any() else moving
    separation = np.where(candidates, distance, np.inf) if candidates.any() else distance
    t, b, y = np.unravel_index(np.argmin(separation), separation.shape)
    return int(t), int(b), int(y + 5)


def closeup(rec, event):
    tick, blue, yellow = event
    xy = positions(rec)
    width, height = 672, 354
    first = max(0, tick - 48) // 2 * 2
    last = min(len(xy) - 1, tick + 48) // 2 * 2
    ids = [rec[0]["keys"][k][1] for k in (blue, yellow)]
    span = np.abs(xy[first : last + 1, blue] - xy[first : last + 1, yellow]).max(axis=0)
    size = max(4.2, span[0] + 1.1, (span[1] + 0.8) * width / height)

    def point(p):
        u, v = (p * (16.5 / (16.5 - 0.15)) - center) * width / size
        return width / 2 + u, height / 2 - v

    for t in range(first, last + 1, 2):
        center = (xy[t, blue] + xy[t, yellow]) / 2
        im = Image.new("RGB", (width, 454), BG)
        d = ImageDraw.Draw(im)
        d.text((12, 8), f"Blue {ids[0]} meets Yellow {ids[1]}", font=FONTS(25), fill=WHITE)
        d.text(
            (width - 12, 15),
            f"{t * CFG.dt:.2f} s   ·   real time",
            anchor="ra",
            font=FONTS(16),
            fill=WHITE,
        )
        board = pitch(rec, t, center, size, (width, height), ())
        ink = ImageDraw.Draw(board)
        for k, color in ((blue, BLUE), (yellow, YELLOW)):
            pts = [point(p) for p in xy[max(0, t - 13) : t + 1, k]]
            if len(pts) > 1:
                ink.line(pts, fill=color, width=3)
            a, b = pts[-1]
            ink.ellipse((a - 12, b - 12, a + 12, b + 12), outline=color, width=2)
        # Reconstruct the same float32 millimetre observations that Blue received.
        obs = (xy[[max(0, t - 1), t], yellow] * 1000).astype(np.float32).astype(float) / 1000
        start, end = obs[1], obs[1] + STEERING.horizon * (obs[1] - obs[0]) / CFG.dt
        for f in np.arange(0, 1, 0.20):
            ink.line(
                [point(start + f * (end - start)), point(start + (f + 0.10) * (end - start))],
                fill="#ffb4dc",
                width=3,
            )
        im.paste(board, (0, 47))
        gap = np.linalg.norm(xy[t, blue] - xy[t, yellow])
        d.text(
            (12, 415),
            "Pink dashes   Yellow’s observed velocity projected 0.62 s",
            font=FONTS(16),
            fill="#ffb4dc",
        )
        d.text((width - 12, 415), f"{gap:.2f} m apart", anchor="ra", font=FONTS(16), fill=WHITE)
        yield im


def film(folder, cases, output):
    case = next(c for c in cases if c["kind"] == "half-time")
    manifest = json.loads((folder / "kickoff-replays.json").read_text())
    records = []
    with np.load(folder / "kickoff-traces.npz") as traces:
        for arm in TEAM_ARMS:
            row = next(
                r
                for r in manifest
                if (r["seed"], r["restart"], r["arm"]) == (case["seed"], case["restart"], arm)
            )
            assert row["ready"]
            images = sorted((folder / "replay-frames" / row["name"]).glob("*.jpg"))
            records.append(native_camera(dict(keys=case["keys"]), traces[row["name"]], row, images))
    np.testing.assert_allclose(records[0][1][0], records[1][1][0], rtol=0, atol=1e-9)
    output.mkdir(parents=True, exist_ok=True)
    save_clip(output / "matched-kickoffs.mp4", comparison(records))
    event = encounter(records[1])
    save_clip(output / "observed-opponent.mp4", closeup(records[1], event))
    t, b, y = event
    print(
        dict(
            seed=case["seed"],
            restart=case["restart"],
            seconds=t * CFG.dt,
            blue=case["keys"][b][1],
            yellow=case["keys"][y][1],
            gap_m=float(np.linalg.norm(positions(records[1])[t, b] - positions(records[1])[t, y])),
        )
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("replays", type=Path)
    p.add_argument("cases", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    film(args.replays, json.loads(args.cases.read_text()), args.output)
