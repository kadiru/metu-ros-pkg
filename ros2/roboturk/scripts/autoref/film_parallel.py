"""Compare the native half-time cameras at the same elapsed time after each award."""

import argparse
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw

from film_kickoff import halfway
from film_league import FONTS
from film_open_motion import BG, WHITE, BLUE, YELLOW, encode, pitch
from run_league import PARALLEL_ARMS

W, H = 464, 286


def film(pair, output):
    output.mkdir(parents=True, exist_ok=True)
    cases = [halfway(pair / arm, True) for arm in PARALLEL_ARMS]
    duration = max(award["native_s"] for _, _, award in cases)

    def frames():
        # Native frames are captured at 12.5 Hz. Four camera frames per output frame gives 4x.
        for sec in np.arange(0, duration + 1.28, 0.32):
            im = Image.new("RGB", (2 * W + 4, 418), BG)
            d = ImageDraw.Draw(im)
            d.text((12, 8), "How fast can ten robots change ends?", font=FONTS(24), fill=WHITE)
            d.text(
                (im.width - 14, 15),
                f"{sec:.1f} s from award    ·    4×",
                anchor="ra",
                font=FONTS(16),
                fill=WHITE,
            )
            for index, (rec, _, award) in enumerate(cases):
                x = index * (W + 4)
                dt = rec[0]["dt"]
                t = min(award["end"], award["start"] + round(sec / dt)) // 2 * 2
                xy = rec[1][t, 4:].reshape(10, 3)[:, :2]
                old = rec[1][max(0, t - 1), 4:].reshape(10, 3)[:, :2]
                moving = np.flatnonzero(np.linalg.norm(xy - old, axis=1) > 0.2 * dt)
                board = pitch(rec, t, (0, 0), 10, (W, H), ())
                ink = ImageDraw.Draw(board)
                for k in moving:
                    unit = W / 10 * 16.5 / (16.5 - 0.15)
                    track = rec[1][max(award["start"], t - 14) : t + 1, 4 + 3 * k : 6 + 3 * k]
                    pts = [(W / 2 + unit * u, H / 2 - unit * v) for u, v in track]
                    if len(pts) > 1:
                        ink.line(pts, fill=BLUE if k < 5 else YELLOW, width=2)
                d.text(
                    (x + 12, 49),
                    ("Current planner", "Concurrent routes")[index],
                    font=FONTS(21),
                    fill=WHITE,
                )
                im.paste(board, (x, 82))
                released = sec >= award["native_s"]
                label = (
                    f"Kickoff released at {award['native_s']:.2f} s  ·  held"
                    if released
                    else f"{len(moving)} of 10 robots moving"
                )
                d.text((x + 12, 381), label, font=FONTS(18), fill="#9fe4bf" if released else WHITE)
            yield im

    movie = output / "kickoff-race.mp4"
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
            "fps=12.5,scale=880:-2,split[a][b];[a]palettegen[p];[b][p]paletteuse=dither=bayer",
            "-loop",
            "0",
            str(output / "kickoff-race.gif"),
        ],
        check=True,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pair", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    film(args.pair, args.output)
