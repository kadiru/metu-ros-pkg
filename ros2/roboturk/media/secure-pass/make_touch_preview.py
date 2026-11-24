"""Show each native receiver close up on a common clock around first touch."""

import argparse
import json
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def load(movie, temp):
    data = json.loads(movie.with_suffix(".json").read_text())
    (row,) = data["episodes"]
    if data.get("source") != "actual Webots movie recorder":
        raise ValueError("Expected a Webots recording")
    transfer = row["transfer"]
    touch = transfer["first_receiver_touch"]["seconds"]
    end = transfer["elapsed_seconds"]
    # Synchronize on the pass release so an earlier first touch remains visible.
    release = transfer["pass_step"] * transfer["elapsed_seconds"] / transfer["steps"]
    start = max(0, int((release - 0.08) * 25))
    last = int(np.ceil(end * 25))
    temp.mkdir()
    subprocess.run(
        [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-i",
            str(movie),
            "-vf",
            f"select=between(n\\,{start}\\,{last})",
            "-fps_mode",
            "passthrough",
            str(temp / "%04d.png"),
        ],
        check=True,
    )
    return row, release, touch, start, sorted(temp.glob("*.png"))


def crop_cyan(path):
    with Image.open(path) as image:
        scene = image.convert("RGB")
    red, green, blue = np.asarray(scene, dtype=float).transpose(2, 0, 1)
    mask = (green > 85) & (blue > 85) & (red < 0.5 * green) & (abs(blue - green) < 60)
    ys, xs = np.where(mask)
    if len(xs) < 80:
        raise ValueError(f"Receiver missing in {path}")
    # The body contains far more cyan pixels than the native phase label.
    x, y = int(np.median(xs)), int(np.median(ys))
    x = min(max(x - 200, 0), scene.width - 400)
    y = min(max(y - 126, 0), scene.height - 252)
    return scene.crop((x, y, x + 400, y + 252)).resize((480, 302), Image.Resampling.LANCZOS)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("baseline", type=Path)
    p.add_argument("trained", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    with TemporaryDirectory() as directory:
        sides = [
            load(movie, Path(directory) / str(i))
            for i, movie in enumerate((args.baseline, args.trained))
        ]
        if sides[0][0]["transfer"]["opening"] != sides[1][0]["transfer"]["opening"]:
            raise ValueError("Different openings")
        images = []
        font = ImageFont.load_default(size=19)
        for frame in range(max(len(side[-1]) for side in sides) + 8):
            output = Image.new("RGB", (963, 367), "#11232c")
            draw = ImageDraw.Draw(output)
            for i, (row, release, touch, start, frames) in enumerate(sides):
                index = min(frame, len(frames) - 1)
                time = (start + index) / 25
                transfer = row["transfer"]
                end = transfer["elapsed_seconds"]
                phase = "Tracking the pass"
                if time >= touch:
                    phase = "Ball at the roller"
                if time >= end:
                    phase = {
                        "received": "Stable possession",
                        "defender_collision": "Defender too close",
                        "defender_interception": "Yellow reached the ball",
                    }.get(transfer["reason"], transfer["reason"].replace("_", " "))
                x = 483 * i
                draw.text(
                    (x + 12, 5),
                    ("Previous receiver", "Trained receiver")[i],
                    font=font,
                    fill="white",
                )
                draw.text((x + 468, 6), "Half speed", font=font, fill="#d1dbe0", anchor="rt")
                draw.text((x + 12, 34), phase, font=font, fill="#ffcf78")
                draw.text(
                    (x + 468, 35),
                    f"{min(time, end) - release:+.2f} s",
                    font=font,
                    fill="white",
                    anchor="rt",
                )
                output.paste(crop_cyan(frames[index]), (x, 65))
            images.append(output)
        images[0].save(args.output, save_all=True, append_images=images[1:], duration=80, loop=0)


if __name__ == "__main__":
    main()
