"""Show the two native half-time formations with the real TIGERs keeper estimate."""

import argparse
import math
from pathlib import Path
import subprocess
from PIL import Image, ImageDraw
from audit_kickoff import measure, tracked_positions
from film_league import FONTS
from film_open_motion import BG, WHITE, encode, pitch, recording

WIDTH, HEIGHT = 432, 280
TRACK = "#fface4"
NATIVE = "#94dcff"


def halfway(folder, driven):
    result, _, _ = measure(folder, driven)
    rec = recording(folder / "match", result)
    first = next(
        f
        for f in result["formations"]
        if rec[0]["frames"][f["start"]]["stage"] == "NORMAL_SECOND_HALF_PRE"
    )
    return rec, tracked_positions(rec[0]), first


def point(x, y):
    unit = WIDTH / 10 * 16.5 / (16.5 - 0.15)
    return WIDTH / 2 + unit * x, HEIGHT / 2 - unit * y


def panel(case, elapsed):
    rec, tracker, award = case
    frames = rec[0]["frames"]
    dt = rec[0]["dt"]
    # Keep one second of play on the screen following each independent release.
    last = min(len(frames) - 1, award["end"] + round(1 / dt))
    t = min(last, max(0, award["start"] + round(elapsed / dt))) // 2 * 2
    image = pitch(rec, t, (0, 0), 10, (WIDTH, HEIGHT), ())
    d = ImageDraw.Draw(image)
    k = rec[0]["keys"].index(["blue", 0])
    x, y = rec[1][t, 4 + 3 * k : 6 + 3 * k]
    cx, cy = point(x, y)
    tx, ty = tracker[t, k]
    error = "Keeper untracked"
    if math.isfinite(tx + ty):
        a, b = point(tx, ty)
        d.line((cx, cy, a, b), fill=TRACK, width=1)
        d.line((a - 6, b, a + 6, b), fill=TRACK, width=2)
        d.line((a, b - 6, a, b + 6), fill=TRACK, width=2)
        error = f"Tracker error   {math.hypot(x - tx, y - ty):.2f} m"
    d.ellipse((cx - 7, cy - 7, cx + 7, cy + 7), outline=NATIVE, width=2)
    phase = (
        "Frame held after kickoff"
        if award["start"] + round(elapsed / dt) > last
        else frames[t]["ssl_command"].replace("_", " ").title()
    )
    return image, error, phase


def film(pair, output):
    output.mkdir(parents=True, exist_ok=True)
    cases = [halfway(pair / arm, arm == "drive") for arm in ("reset", "drive")]
    end = max(c[2]["native_s"] for c in cases) + 1.2
    times = [-0.8 + 0.08 * i for i in range(36)]
    while times[-1] < end:
        times.append(times[-1] + 0.32)

    def frames():
        for seconds in times:
            im = Image.new("RGB", (2 * WIDTH + 4, 424), BG)
            d = ImageDraw.Draw(im)
            d.text((14, 9), "Changing ends at half-time", font=FONTS(24), fill=WHITE)
            speed = "1×" if seconds <= 2.001 else "4×"
            d.text(
                (im.width - 14, 13),
                f"{seconds:+.1f} s from award    ·    {speed}",
                anchor="ra",
                font=FONTS(17),
                fill=WHITE,
            )
            for n, case in enumerate(cases):
                board, error, phase = panel(case, seconds)
                x = n * (WIDTH + 4)
                d.text(
                    (x + 12, 51),
                    ("Instant reset", "Robots drive to position")[n],
                    font=FONTS(19),
                    fill=WHITE,
                )
                im.paste(board, (x, 80))
                d.text((x + 12, 366), error, font=FONTS(17), fill=WHITE)
                d.text((x + WIDTH - 10, 368), phase, anchor="ra", font=FONTS(13), fill=WHITE)
            d.text((14, 398), "Blue ring   Blue 0 in Webots", font=FONTS(16), fill=NATIVE)
            d.text((WIDTH + 16, 398), "Pink cross   Blue 0 in TIGERs", font=FONTS(16), fill=TRACK)
            yield im

    movie = output / "changing-ends.mp4"
    encode(movie, frames(), 12.5)
    subprocess.run(
        [
            "ffmpeg",
            "-v",
            "error",
            "-y",
            "-i",
            str(movie),
            "-vf",
            "fps=12.5,scale=800:-2,split[a][b];[a]palettegen[p];[b][p]paletteuse=dither=bayer",
            "-loop",
            "0",
            str(output / "changing-ends.gif"),
        ],
        check=True,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pair", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    film(args.pair, args.output)
