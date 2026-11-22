"""Show actual release choices from independently verified fresh matches."""

import argparse
import tempfile
from pathlib import Path
from PIL import Image, ImageDraw
from annotate_blocked_outlet import font, pixel
from build_contest_preview import digest, ffmpeg, make_gif
from compare_takeaways import trace
from film_counterpass import tracking_crop
from train_pass_release import read, save


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("demo", type=Path)
    p.add_argument("media", type=Path)
    a = p.parse_args()
    case = read(a.demo / "full.json")
    clips = [dict(folder="full", seed=case["seed"], decision=d) for d in case["clips"]]
    clips.extend(
        dict(folder=e["folder"], seed=e["seed"], decision=e["release_decision"])
        for e in case["extras"]
    )
    clips.sort(key=lambda clip: clip["decision"]["action"])
    assert clips
    receipts = {}
    a.media.mkdir(parents=True, exist_ok=True)
    movie = a.media / "release-choices.mp4"
    with tempfile.TemporaryDirectory() as directory:
        tmp = Path(directory)
        n = 0
        for clip in clips:
            d = clip["decision"]
            replay = a.demo / clip["folder"]
            check = read(replay / "replay.json")
            assert (
                check["exact_events"]
                and check["exact_trajectory"]
                and check["seed"] == clip["seed"]
            )
            receipts[clip["folder"]] = check
            game = read(replay / "game-000.json")
            s = trace(replay / "game-000.json")
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
                attack_x = game["initial_world"]["attack_sign"] * (1 if d["team"] == "blue" else -1)
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
                        field.crop(
                            tracking_crop(s, min(round(t * 100), d["end"]), attack_x, 490, 296)
                        ).resize((768, 464), Image.Resampling.LANCZOS),
                        (0, 40),
                    )
                    draw = ImageDraw.Draw(img)
                    draw.text((12, 5), title, font=font(25), fill="white")
                    draw.text(
                        (754, 9),
                        f"Kickoff {game['seed']}  /  {t:.2f} s",
                        font=font(18),
                        anchor="ra",
                        fill="white",
                    )
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
        dict(choices=clips, replays=receipts, sha256=digest(movie), gif=gif),
    )


if __name__ == "__main__":
    main()
