"""Encode synchronized Webots recordings of one opening contest (Pillow is used for titles)."""

import argparse
import json
from pathlib import Path
import tempfile
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from build_contest_preview import ffmpeg, make_gif, digest


def main():
    p = argparse.ArgumentParser()
    p.add_argument("pair", type=Path)
    p.add_argument("media", type=Path)
    a = p.parse_args()
    a.media.mkdir(exist_ok=True, parents=True)
    evidence = json.loads((a.pair / "replay.json").read_text())
    before = evidence["before"]
    previous_end = before.get("recovery_s", 0) + before.get("elapsed_s", 0)
    goal_time = evidence["play"]["goal"] * 0.01
    end = min(24.0, max(previous_end, goal_time) + 1.0)

    # Identical crop for both views. Include the entire scored attack and the original turnover.
    pos = []
    for name in ("previous", "trained"):
        with np.load(a.pair / name / "trace-000.npz") as t:
            stop = previous_end if name == "previous" and previous_end else goal_time
            pos.extend(t["states"][: round(stop * 100) + 1, :2])
    pixels = (np.array(pos) * [0.073, -0.130] + [0.5, 0.5]) * [1280, 720]
    lo, hi = pixels.min(0), pixels.max(0)
    width = min(1280, 2 * int(max(470, hi[0] - lo[0] + 145, (hi[1] - lo[1] + 130) * 16 / 9) / 2))
    height = min(720, 2 * int(width * 9 / 32))
    cx, cy = (lo + hi) / 2
    x = int(np.clip(cx - width * 0.5, 0, 1280 - width))
    y = int(np.clip(cy - height * 0.5, 0, 720 - height))
    banner = Image.new("RGB", (1280, 74), "#162b24")
    draw = ImageDraw.Draw(banner)
    try:
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 25)
    except OSError:
        font = ImageFont.load_default(size=25)
    draw.text((18, 6), "Previous team", font=font, fill="white")
    draw.text((658, 6), "New self-play policy", font=font, fill="white")
    small = ImageFont.load_default(size=22)
    label = before["outcome"].capitalize()
    if previous_end:
        label += f" at {previous_end:.2f} s"
    draw.text((18, 41), label, font=small, fill="#d5e3d9")
    draw.text((658, 41), f"Recovery goal at {goal_time:.2f} s", font=small, fill="#d5e3d9")
    with tempfile.TemporaryDirectory() as tmp:
        title = Path(tmp) / "title.png"
        banner.save(title)
        movie = a.media / "matched-recovery.mp4"
        args = [
            "-i",
            a.pair / "previous/match.mp4",
            "-i",
            a.pair / "trained/match.mp4",
            "-i",
            title,
        ]
        views = [
            f"[{i}:v]crop={width}:{height}:{x}:{y},scale=640:360,setsar=1[v{i}]" for i in range(2)
        ]
        ffmpeg(
            *args,
            "-filter_complex_threads",
            "1",
            "-filter_complex",
            ";".join(
                views + ["[v0][v1]hstack,pad=1280:434:0:74[game]", "[game][2:v]overlay=0:0[v]"]
            ),
            "-map",
            "[v]",
            "-t",
            end,
            "-an",
            "-c:v",
            "libx264",
            "-crf",
            "16",
            "-pix_fmt",
            "yuv420p",
            movie,
        )
        gif = make_gif(movie, movie.with_suffix(".gif"), 0, end, 12.5)
    evidence.update(
        excerpt=gif,
        mp4_sha256=digest(movie),
        speed=1,
        source_start=0,
        source_end=end,
        common_crop=[x, y, width, height],
        native_sources={k: digest(a.pair / k / "match.mp4") for k in ("previous", "trained")},
    )
    movie.with_suffix(".json").write_text(json.dumps(evidence, indent=2) + "\n")


if __name__ == "__main__":
    main()
