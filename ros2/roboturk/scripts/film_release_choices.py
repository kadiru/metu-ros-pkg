"""Show actual release choices from a single independently verified fresh match."""

import argparse
import tempfile
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
from annotate_blocked_outlet import font, pixel
from build_contest_preview import digest, ffmpeg, make_gif
from compare_takeaways import trace
from train_pass_release import read, save


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("demo", type=Path)
    p.add_argument("media", type=Path)
    a = p.parse_args()
    case = read(a.demo / "full.json")
    replay = a.demo / "full"
    check = read(replay / "replay.json")
    assert check["exact_events"] and check["exact_trajectory"]
    game = read(replay / "game-000.json")
    s = trace(replay / "game-000.json")
    clips = case["clips"]
    assert clips and game["seed"] == case["seed"]
    a.media.mkdir(parents=True, exist_ok=True)
    movie = a.media / "release-choices.mp4"
    with tempfile.TemporaryDirectory() as directory:
        tmp = Path(directory)
        n = 0
        for d in clips:
            title = {1: "Delay the pass", 2: "Switch receivers", 3: "Keep dribbling"}[d["action"]]
            team = d["team"].title()
            actor = d["actor"]
            start = max(0, 0.04 * d["tick"] - 0.65)
            end = min(90, (d["end"] + 1) / 100 + 0.4, start + 8)
            if d["release_tick"] is not None:
                delivery = next(
                    p
                    for p in game["passes"]
                    if p["tick"] == d["release_tick"] and p["team"] == d["team"]
                )
            else:
                delivery = None
            with tempfile.TemporaryDirectory(dir=tmp) as part:
                frames = Path(part)
                ffmpeg(
                    "-ss",
                    start,
                    "-i",
                    replay / "match.mp4",
                    "-t",
                    end - start,
                    "-vf",
                    "fps=12.5",
                    "-start_number",
                    0,
                    frames / "f-%04d.png",
                )
                # Keep the same crop throughout each play. Include every recorded ball position.
                loc = np.array(
                    [pixel(v) for v in s[round(100 * start) : min(len(s), d["end"] + 1), :2]]
                )
                low, high = loc.min(0), loc.max(0)
                width = min(960, max(470, high[0] - low[0] + 140, (high[1] - low[1] + 95) * 1.65))
                height = round(width / 1.65)
                x = round(np.clip((low[0] + high[0] - width) / 2, 160, 1120 - width))
                y = round(np.clip((low[1] + high[1] - height) / 2, 55, 657 - height))
                for j, f in enumerate(sorted(frames.glob("f-*.png"))):
                    t = start + j / 12.5
                    with Image.open(f) as src:
                        field = src.convert("RGB")
                    if t * 100 <= d["end"]:
                        bx, by = pixel(s[min(len(s) - 1, round(t * 100)), :2])
                        draw = ImageDraw.Draw(field)
                        draw.ellipse(
                            (bx - 10, by - 10, bx + 10, by + 10), outline="#f4cb76", width=2
                        )
                    img = Image.new("RGB", (768, 548), (17, 30, 39))
                    img.paste(
                        field.crop((x, y, x + round(width), y + height)).resize(
                            (768, 464), Image.Resampling.LANCZOS
                        ),
                        (0, 40),
                    )
                    draw = ImageDraw.Draw(img)
                    draw.text((12, 5), title, font=font(25), fill="white")
                    draw.text((754, 9), f"{t:.2f} s", font=font(18), anchor="ra", fill="white")
                    caption = f"{team} {actor} prepares a pass to {d['intended']}"
                    if t >= d["tick"] * 0.04:
                        caption = {
                            1: f"{team} {actor} holds the ball for {d['intended']}",
                            2: f"{team} {actor} changes the target to {d['alternative']}",
                            3: f"{team} {actor} carries the ball",
                        }[d["action"]]
                    if delivery and t >= delivery["tick"] * 0.04:
                        caption = f"{team} {actor} passes to {delivery['receiver']}"
                    if delivery and t >= (delivery["end"] + 1) * 0.04:
                        caption = (
                            f"{team} {delivery['receiver']} controls the ball"
                            if delivery["result"] == "complete"
                            else f"Pass outcome — {delivery['result']}"
                        )
                    if d["goal"] is not None and t >= (d["goal"] + 1) / 100:
                        caption = f"Goal for {team}"
                    draw.text((12, 509), caption, font=font(23), fill="#f4cb76")
                    img.save(tmp / f"choice-{n:04d}.png")
                    n += 1
        ffmpeg(
            "-framerate",
            12.5,
            "-i",
            tmp / "choice-%04d.png",
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-crf",
            18,
            movie,
        )
    gif = make_gif(movie, movie.with_suffix(".gif"), 0, n / 12.5, 12.5)
    save(
        a.media / "release-choices.json",
        dict(seed=game["seed"], choices=clips, replay=check, sha256=digest(movie), gif=gif),
    )


if __name__ == "__main__":
    main()
