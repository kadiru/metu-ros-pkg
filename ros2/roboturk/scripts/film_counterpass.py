"""Cut a synchronized comparison of two verified Webots shots from one recovery."""

import argparse
from pathlib import Path
import tempfile
import numpy as np
from PIL import Image, ImageDraw
from annotate_blocked_outlet import font, pixel
from build_contest_preview import digest, ffmpeg, make_gif
from compare_takeaways import trace
from train_counterpass import read, save


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("demo", type=Path)
    p.add_argument("media", type=Path)
    a = p.parse_args()
    a.media.mkdir(parents=True, exist_ok=True)
    case = read(a.demo / "selected.json")
    prior = read(a.demo / "previous/replay.json")
    current = read(a.demo / "trained/replay.json")
    assert all(x["exact_events"] and x["exact_trajectory"] for x in (prior, current))
    sides = []
    for key, r, label in [
        ("previous", case["old"], "73-goal finish"),
        ("trained", case["trained"], "Learned team decision"),
    ]:
        folder = a.demo / key
        g = read(folder / "game-000.json")
        s = trace(folder / "game-000.json")
        shot = next(
            e
            for e in reversed(g["events"])
            if e["event"] == "kick"
            and e["team"] == r["team"]
            and e["speed"] == 4
            and r["sample"] <= 4 * e["tick"] <= r["end"]
        )
        sides.append((folder, r, label, g, s, shot))
    through = case["identical_samples"]
    assert np.array_equal(sides[0][4][:through], sides[1][4][:through])
    start = max(0, (case["sample"] + 1) / 100 - 1.0)
    end = min(90, max(x[1]["end"] + 1 for x in sides) / 100 + 1.0)
    fps = 12.5
    team = case["trained"]["team"].title()
    file = a.media / "same-possession.mp4"
    with tempfile.TemporaryDirectory() as work:
        path = Path(work)
        for k, (folder, r, label, g, s, shot) in enumerate(sides):
            ffmpeg(
                "-ss",
                start,
                "-i",
                folder / "match.mp4",
                "-t",
                end - start,
                "-vf",
                "fps=12.5",
                "-start_number",
                0,
                path / f"p{k}-%04d.png",
            )
        n = min(len(list(path.glob(f"p{k}-*.png"))) for k in range(2))
        for j in range(n):
            t = start + j / fps
            frame = Image.new("RGB", (1152, 442), (17, 30, 39))
            d = ImageDraw.Draw(frame)
            for k, (_, r, label, g, s, shot) in enumerate(sides):
                sec = min(t, (r["end"] + 1) / 100 - 0.04)
                idx = max(0, min(j, round((sec - start) * fps)))
                img = Image.open(path / f"p{k}-{idx:04}.png").convert("RGB")
                # The yellow ring follows the actual recorded ball in the native camera.
                bx, by = pixel(s[max(0, round(sec * 100)), :2])
                mark = ImageDraw.Draw(img)
                mark.ellipse((bx - 15, by - 15, bx + 15, by + 15), outline="#f4cb76", width=2)
                img = img.crop((176, 64, 1104, 656)).resize((566, 360), Image.Resampling.LANCZOS)
                x = k * 576 + 5
                frame.paste(img, (x, 38))
                d.text((x + 7, 5), label, font=font(23), fill="white")
                d.text(
                    (x + 558, 7),
                    f"{min(t, (r['end'] + 1) / 100):.2f} s",
                    font=font(18),
                    anchor="ra",
                    fill="white",
                )
                caption = "Same possession"
                if 100 * t >= r["sample"] + 1:
                    caption = f"{team} {r['winner'] % 3} wins the ball"
                for ps in (p for p in g["passes"] if p["tick"] in r.get("passes", [])):
                    if 100 * t >= 4 * ps["tick"]:
                        caption = f"{team} {ps['passer']} leads {team} {ps['receiver']}"
                    if 100 * t >= 4 * (ps["end"] + 1):
                        caption = f"{team} {ps['receiver']} controls the pass"
                if 100 * t >= 4 * shot["tick"]:
                    caption = f"{team} {shot['robot']} shoots"
                if 100 * t >= r["end"] + 1:
                    caption = {
                        "goal": f"Goal for {team}",
                        "opponent touch": "Opponent touches the ball",
                        "ball out": "Shot goes out",
                    }[r["outcome"]]
                d.text((x + 8, 402), caption, font=font(21), fill="#f4cb76")
            frame.save(path / f"frame-{j:04}.png")
        ffmpeg(
            "-framerate",
            fps,
            "-i",
            path / "frame-%04d.png",
            "-c:v",
            "libx264",
            "-crf",
            "18",
            "-pix_fmt",
            "yuv420p",
            file,
        )
        gif = make_gif(file, file.with_suffix(".gif"), 0, n / fps, fps)
    case = {k: v for k, v in case.items() if not k.endswith("_folder")}
    save(
        a.media / "same-possession.json",
        dict(case=case, trained=current, previous=prior, sha256=digest(file), gif=gif),
    )


if __name__ == "__main__":
    main()
