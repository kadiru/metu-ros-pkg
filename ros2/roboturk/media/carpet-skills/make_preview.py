"""Preview native one-episode team-demo movies with their measured phase times."""

import argparse
import json
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

import numpy as np
from PIL import Image, ImageDraw, ImageFont

FRAME_SECONDS = 0.08


def read_trial(movie):
    data = json.loads(movie.with_suffix(".json").read_text())
    (row,) = data["episodes"]
    if data["source"] != "actual Webots movie recorder":
        raise ValueError("Expected a native team-demo recording")
    transfer, shot = row["transfer"], row["shooting"]
    dt = transfer["elapsed_seconds"] / transfer["steps"]
    events = [(0, "Lining up the pass")]
    if transfer["passed"]:
        events.append((transfer["pass_step"] * dt, "Awaiting reception"))
    if shot:
        events.append((transfer["elapsed_seconds"], "Received · carrying to goal"))
        if shot["shot"]:
            events.append((transfer["elapsed_seconds"] + shot["shot_seconds"], "Shooting"))
    outcome = {"goal": "Goal", "timeout": "Timed out"}.get(row["reason"], row["reason"])
    events.append((row["elapsed_seconds"], f"{outcome} at {row['elapsed_seconds']:.2f} s"))
    return row, events


def native_frames(movie, folder, width):
    folder.mkdir()
    subprocess.run(
        [
            "ffmpeg",
            "-loglevel",
            "error",
            "-i",
            str(movie),
            "-vf",
            f"fps=12.5,scale={width}:-2:flags=lanczos",
            str(folder / "%06d.png"),
        ],
        check=True,
    )
    return sorted(folder.glob("*.png"))


def mark_ball(frame):
    r, g, b = np.asarray(frame).astype(float).transpose(2, 0, 1)
    y, x = np.where((r > 85) & (r > 1.45 * g) & (g > 1.3 * b))
    # Mark only a localized cluster of orange pixels actually visible in the movie.
    if len(x) >= 3 and np.ptp(x) < 14 and np.ptp(y) < 14:
        cx, cy = np.mean(x), np.mean(y)
        ImageDraw.Draw(frame).ellipse((cx - 8, cy - 8, cx + 8, cy + 8), outline="#ffcc63", width=2)


def preview(movie, output, baseline=None):
    movies = [baseline, movie] if baseline else [movie]
    width = 480 if baseline else 800
    trials = [read_trial(path) for path in movies]
    if len({row["seed"] for row, _ in trials}) != 1:
        raise ValueError("Side-by-side recordings require the same opening")
    font = ImageFont.load_default(size=20)
    small = ImageFont.load_default(size=16)
    frames = []
    with TemporaryDirectory() as temp:
        clips = [native_frames(path, Path(temp) / str(i), width) for i, path in enumerate(movies)]
        # Begin a paired replay just before the earlier release. Single demos show the full episode.
        start = (
            max(0, int((min(events[1][0] for _, events in trials) - 0.6) / FRAME_SECONDS))
            if baseline
            else 0
        )
        for index in range(start, max(map(len, clips))):
            seconds = index * FRAME_SECONDS
            panels = []
            for i, (clip, (row, events)) in enumerate(zip(clips, trials, strict=True)):
                with Image.open(clip[min(index, len(clip) - 1)]) as original:
                    scene = original.convert("RGB")
                mark_ball(scene)
                panel = Image.new("RGB", (width, scene.height + 65), "#10212b")
                panel.paste(scene, (0, 65))
                draw = ImageDraw.Draw(panel)
                title = (
                    ("Original receiver" if i == 0 else "Fitted receiver")
                    if baseline
                    else "Pass, receive and score"
                )
                draw.text((12, 6), title, font=font, fill="white")
                phase = next(
                    label for start_time, label in reversed(events) if seconds >= start_time
                )
                draw.text((12, 35), phase, font=small, fill="#ffc863")
                draw.text(
                    (width - 12, 7),
                    f"{min(seconds, row['elapsed_seconds']):.1f} s",
                    font=small,
                    fill="white",
                    anchor="rt",
                )
                panels.append(panel)
            frame = Image.new(
                "RGB", (width * len(panels) + 3 * (len(panels) - 1), panels[0].height), "white"
            )
            for i, panel in enumerate(panels):
                frame.paste(panel, (i * (width + 3), 0))
            frames.append(frame)
    frames[0].save(output, save_all=True, append_images=frames[1:], duration=80, loop=0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("movie", type=Path)
    parser.add_argument("gif", type=Path)
    parser.add_argument("--baseline", type=Path)
    args = parser.parse_args()
    preview(args.movie, args.gif, args.baseline)
