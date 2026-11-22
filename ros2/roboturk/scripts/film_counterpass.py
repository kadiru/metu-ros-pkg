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
    decision = case.get("release_decision")
    sides = []
    for key, r, label in [
        ("previous", case["old"], "248-goal team" if decision else "73-goal finish"),
        ("trained", case["trained"], "New release policy" if decision else "Learned team decision"),
    ]:
        folder = a.demo / key
        g = read(folder / "game-000.json")
        s = trace(folder / "game-000.json")
        shot = next(
            (
                e
                for e in reversed(g["events"])
                if e["event"] == "kick"
                and e["team"] == r["team"]
                and e["speed"] == 4
                and r["sample"] <= 4 * e["tick"] <= r["end"]
            ),
            None,
        )
        sides.append((folder, r, label, g, s, shot))
    through = case["identical_samples"]
    assert np.array_equal(sides[0][4][:through], sides[1][4][:through])
    start = max(0, (case["sample"] + 1) / 100 - 1.0)
    if decision:
        start = max(start, 0.04 * decision["tick"] - 0.85)
    end = min(90, max(x[1]["end"] + 1 for x in sides) / 100 + 1.0)
    # Lock both cameras to the same region, including both ball paths and the goal.
    points = [
        pixel(point)
        for _, r, _, _, states, _ in sides
        for point in states[round(start * 100) : r["end"] + 1, :2]
    ]
    goal_x = 4.5 if case["trained"]["team"] == "blue" else -4.5
    if case["seed"] % 2:
        goal_x = -goal_x
    points.extend(pixel((goal_x, y)) for y in (-0.5, 0.5))
    low, high = np.min(points, axis=0), np.max(points, axis=0)
    width = min(928, max(480, high[0] - low[0] + 90, (high[1] - low[1] + 60) * 566 / 360))
    height = round(width * 360 / 566)
    left = round(np.clip((low[0] + high[0] - width) / 2, 160, 1120 - width))
    top = round(np.clip((low[1] + high[1] - height) / 2, 55, 657 - height))
    crop = (left, top, left + round(width), top + height)
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
                mark.ellipse((bx - 10, by - 10, bx + 10, by + 10), outline="#f4cb76", width=2)
                img = img.crop(crop).resize((566, 360), Image.Resampling.LANCZOS)
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
                if decision and 100 * t >= 4 * decision["tick"]:
                    if k == 0:
                        caption = f"{team} {decision['actor']} passes to {decision['intended']}"
                    else:
                        caption = [
                            "Release the pass",
                            f"Delay the pass to {decision['intended']}",
                            f"Switch from {decision['intended']} to {decision['alternative']}",
                            f"{team} {decision['actor']} keeps dribbling",
                        ][decision["action"]]
                for ps in (p for p in g["passes"] if p["tick"] in r.get("passes", [])):
                    if 100 * t >= 4 * ps["tick"]:
                        caption = f"{team} {ps['passer']} leads {team} {ps['receiver']}"
                    if 100 * t >= 4 * (ps["end"] + 1):
                        caption = f"{team} {ps['receiver']} controls the pass"
                if shot and 100 * t >= 4 * shot["tick"]:
                    caption = f"{team} {shot['robot']} shoots"
                if 100 * t >= r["end"] + 1:
                    caption = {
                        "goal": f"Goal for {team}",
                        "opponent touch": "Pass intercepted"
                        if decision and k == 0
                        else "Opponent touches the ball",
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
