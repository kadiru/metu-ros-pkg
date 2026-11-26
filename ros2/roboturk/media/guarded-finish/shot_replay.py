"""Pair two native recordings at the instant Blue actually kicks the shot."""

import argparse
import json
import math
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

from PIL import Image, ImageDraw, ImageFont


def load(movie, folder):
    data = json.loads(movie.with_suffix(".json").read_text())
    (row,) = data["episodes"]
    shot = row["shooting"]
    if data["source"] != "actual Webots movie recorder" or not shot or not shot["shot"]:
        raise ValueError("Both sides need a recorded, applied shot")
    kick = row["transfer"]["elapsed_seconds"] + shot["shot_seconds"]
    last = math.ceil(row["elapsed_seconds"] * 25)
    first = max(0, round(kick * 25) - 18)
    folder.mkdir()
    subprocess.run(
        [
            "ffmpeg",
            "-v",
            "error",
            "-i",
            str(movie),
            "-vf",
            f"select=between(n\\,{first}\\,{last}),crop=810:490,scale=470:284:flags=lanczos",
            "-fps_mode",
            "passthrough",
            str(folder / "%04d.png"),
        ],
        check=True,
    )
    frames = sorted(folder.glob("*.png"))
    if len(frames) != last - first + 1:
        raise ValueError("Missing shot frames")
    return row, kick, first, frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("previous", type=Path)
    parser.add_argument("trained", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    with TemporaryDirectory() as root:
        sides = [
            load(movie, Path(root) / str(i))
            for i, movie in enumerate((args.previous, args.trained))
        ]
        if sides[0][0]["transfer"]["opening"] != sides[1][0]["transfer"]["opening"]:
            raise ValueError("Expected the same opening")
        title = ImageFont.load_default(size=20)
        detail = ImageFont.load_default(size=16)
        frames = []
        for i in range(max(len(s[-1]) for s in sides) + 9):
            frame = Image.new("RGB", (943, 350), "#11232c")
            draw = ImageDraw.Draw(frame)
            for j, (row, kick, first, clip) in enumerate(sides):
                idx = min(i, len(clip) - 1)
                t = (first + idx) / 25
                x = 473 * j
                label = "Blue lines up the shot" if t < kick else "Ball in flight"
                if t >= row["elapsed_seconds"] - 1e-8:
                    label = {"goal": "Goal", "keeper_save": "Yellow saves"}.get(
                        row["reason"], row["reason"].replace("_", " ").capitalize()
                    )
                draw.text(
                    (x + 10, 6),
                    ("Previous Blue finish", "Trained Blue finish")[j],
                    font=title,
                    fill="white",
                )
                draw.text((x + 459, 8), "Half speed", font=detail, fill="#cad8e2", anchor="rt")
                draw.text((x + 10, 36), label, font=detail, fill="#ffcf79")
                draw.text(
                    (x + 459, 36),
                    f"{min(t, row['elapsed_seconds']) - kick:+.2f} s from kick",
                    font=detail,
                    fill="white",
                    anchor="rt",
                )
                with Image.open(clip[idx]) as src:
                    frame.paste(src.convert("RGB"), (x, 66))
            frames.append(frame)
        frames[0].save(args.output, save_all=True, append_images=frames[1:], duration=80, loop=0)


if __name__ == "__main__":
    main()
