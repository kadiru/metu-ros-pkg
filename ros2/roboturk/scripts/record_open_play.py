"""Publish native match footage and mark one verified open-play recovery and goal."""

import argparse
from pathlib import Path
import shutil
import tempfile
import numpy as np
from PIL import Image, ImageDraw

from annotate_blocked_outlet import font, pixel
from build_contest_preview import digest, ffmpeg, make_gif
from build_three_preview import probe
from run_recovery_attack import read, save


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path)
    parser.add_argument("media", type=Path)
    args = parser.parse_args()
    args.media.mkdir(parents=True, exist_ok=True)
    replay = read(args.replay / "replay.json")
    assert replay["exact_trajectory"] and replay["exact_events"]
    game = read(args.replay / "game-000.json")
    states = np.load(args.replay / "trace-000.npz")["states"]
    r = replay["moment"]
    assert r in game["recoveries"] and r["outcome"] == "goal"
    full = args.media / "open-play-match.mp4"
    shutil.copyfile(args.replay / "match.mp4", full)
    ffmpeg("-i", full, "-f", "null", "-")
    duration = float(probe(full)["format"]["duration"])
    assert abs(duration - len(states) / 100) < 0.1
    full_gif = make_gif(full, args.media / "open-play-match.gif", 0, duration, 8)
    start = max(0, (r["sample"] + 1) / 100 - 1.6)
    end = min(duration, (r["goal"] + 1) / 100 + 1)
    team = r["team"].title()
    players = f"{team} {r['winner'] % 3}"
    shot = next(
        e
        for e in reversed(game["events"])
        if e["event"] == "kick" and e["tick"] * 4 <= r["goal"] and e["speed"] == 4
    )
    if r["passes"]:
        players += f" to {team} {r['scorer']}"
    with tempfile.TemporaryDirectory() as path:
        path = Path(path)
        ffmpeg(
            "-ss",
            start,
            "-i",
            full,
            "-t",
            end - start,
            "-vf",
            "fps=12.5",
            "-start_number",
            0,
            path / "frame-%04d.png",
        )
        files = sorted(path.glob("frame-*.png"))
        for i, f in enumerate(files):
            t = start + i / 12.5
            j = min(round(t * 100), len(states) - 1)
            src = Image.open(f).convert("RGB")
            img = Image.new("RGB", (960, 580), (17, 30, 39))
            draw = ImageDraw.Draw(img)
            # Keep both goals in frame. The inset is an enlargement of the original movie.
            img.paste(
                src.crop((176, 64, 1104, 656)).resize((812, 518), Image.Resampling.LANCZOS),
                (74, 35),
            )
            xy = states[j, :2] if j <= r["goal"] else states[r["goal"], :2]
            x, y = pixel(xy)
            x = int(np.clip(x - 70, 0, 1140))
            y = int(np.clip(y - 44, 0, 632))
            zoom = src.crop((x, y, x + 140, y + 88)).resize((252, 158), Image.Resampling.LANCZOS)
            ix = 682 if x < 640 else 26
            draw.rectangle((ix - 2, 53, ix + 253, 213), fill=(241, 201, 111))
            img.paste(zoom, (ix, 55))
            draw.text(
                (20, 5), f"RoboTurk  /  3v3 open play     {players}", font=font(22), fill="white"
            )
            draw.text((938, 6), f"{t:.1f} s", font=font(21), fill=(241, 201, 111), anchor="ra")
            caption = "Ball in play"
            if j >= r["sample"]:
                caption = f"{team} {r['winner'] % 3} wins sole possession"
            if any(4 * k <= j for k in r["passes"]):
                caption = f"{team} attacks through the pass"
            if j >= shot["tick"] * 4:
                caption = f"{team} {r['scorer']} shoots"
            if j >= r["goal"]:
                caption = f"Goal for {team}"
            draw.text((22, 553), caption, font=font(21), fill="white")
            img.save(f)
        clip = args.media / "open-play-recovery.mp4"
        ffmpeg(
            "-framerate",
            "12.5",
            "-i",
            path / "frame-%04d.png",
            "-c:v",
            "libx264",
            "-crf",
            "19",
            "-pix_fmt",
            "yuv420p",
            clip,
        )
        excerpt = make_gif(clip, args.media / "open-play-recovery.gif", 0, len(files) / 12.5, 12.5)
    save(
        args.media / "recording.json",
        replay
        | dict(
            full_seconds=duration,
            native_sha256=digest(full),
            excerpt_seconds=[start, end],
            highlight_sha256=digest(clip),
            full=full_gif,
            highlight=excerpt,
        ),
    )


if __name__ == "__main__":
    main()
