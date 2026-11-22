"""Synchronize two native recordings and magnify the carrier at the original loss."""

import argparse
from pathlib import Path
import tempfile
import numpy as np
from PIL import Image, ImageDraw
from annotate_blocked_outlet import pixel, font
from build_contest_preview import ffmpeg, make_gif, digest
from run_recovery_attack import read, save


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("pair", type=Path)
    p.add_argument("media", type=Path)
    a = p.parse_args()
    evidence = read(a.pair / "replay.json")
    old, new = evidence["original"], evidence["trained"]
    rows = [old, new]
    names = ["previous", "trained"]
    states = [np.load(a.pair / n / "trace-000.npz")["states"] for n in names]
    duration = (new["chain"]["goal"] + 1) / 100 + 0.8
    loss = (old["first_break"] + 1) / 100
    duration = max(duration, loss + 0.8)
    team = new["chain"]["team"].title()
    carrier = new["chain"]["winner"]
    points = np.concatenate(
        [s[: int(100 * t), :2] for s, t in zip(states, [loss + 0.25, duration], strict=True)]
    )
    pts = np.array([pixel(point) for point in points])
    low, high = pts.min(0), pts.max(0)
    width = min(1280, int(max(430, (high - low)[0] + 120, ((high - low)[1] + 115) * 1.8)))
    height = int(width / 1.8)
    center = (high + low) * 0.5
    x = int(np.clip(center[0] - width / 2, 0, 1280 - width))
    y = int(np.clip(center[1] - height / 2, 0, 720 - height))
    dark = (17, 29, 39)
    white = (240, 245, 246)
    gold = (255, 205, 109)
    green = (124, 236, 202)
    a.media.mkdir(parents=True, exist_ok=True)
    movie = a.media / "first-touch-comparison.mp4"
    with tempfile.TemporaryDirectory() as directory:
        tmp = Path(directory)
        for side in names:
            ffmpeg(
                "-i",
                a.pair / side / "match.mp4",
                "-t",
                duration,
                "-vf",
                "fps=12.5",
                "-start_number",
                0,
                tmp / (side + "-%03d.png"),
            )
        for i, f in enumerate(sorted(tmp.glob("previous-*.png"))):
            sec = i / 12.5
            j = min(round(sec * 100), len(states[0]) - 1)
            frame = Image.new("RGB", (1120, 440), dark)
            draw = ImageDraw.Draw(frame)
            for k, name in enumerate(names):
                left = 560 * k
                s = states[k][j]
                rs = s[4:].reshape(6, 3)
                row = rows[k]
                native = Image.open(tmp / f"{name}-{i:03}.png").convert("RGB")
                field = native.crop((x, y, x + width, y + height)).resize(
                    (554, 308), Image.Resampling.LANCZOS
                )
                frame.paste(field, (left + 3, 63))
                draw.text(
                    (left + 14, 10),
                    "Previous team" if not k else "New first-touch policy",
                    font=font(24),
                    fill=white,
                )
                draw.text(
                    (left + 14, 40),
                    f"{team} {carrier % 3}   /   same midfield opening",
                    font=font(15),
                    fill=white,
                )
                draw.text((left + 537, 15), f"{sec:04.1f} s", font=font(19), fill=gold, anchor="ra")
                if sec <= max(loss + 0.55, row["chain"]["recovery"] / 100 + 0.4):
                    px, py = pixel(rs[carrier, :2])
                    cx = int(np.clip(px - 40, 0, 1200))
                    cy = int(np.clip(py - 27, 0, 666))
                    detail = native.crop((cx, cy, cx + 80, cy + 54)).resize(
                        (144, 97), Image.Resampling.LANCZOS
                    )
                    ix = left + (8 if px > x + width / 2 else 405)
                    draw.rectangle((ix - 2, 68, ix + 145, 168), outline=gold, width=2)
                    frame.paste(detail, (ix, 70))
                caption = "Opening contest"
                if not k and sec >= loss:
                    caption = f"Ball escapes the roller at {loss:.2f} s"
                elif k and j >= row["chain"]["goal"]:
                    caption = f"Goal by {team} {row['receiver']}"
                elif k and j >= 4 * row["chain"]["shot"]:
                    caption = "Receiver shoots"
                elif k and j >= row["shield_end"]:
                    caption = "Receiver shields and turns to goal"
                elif k and j >= row["contact"]:
                    caption = "Receiver controls the lead pass"
                elif k and j >= 4 * row["kick"]:
                    caption = "Lead pass to the running teammate"
                else:
                    if j >= row["chain"]["recovery"]:
                        caption = "Carrier has sole possession"
                    if k:
                        cmd = next(
                            (
                                t
                                for t in evidence["first_touches"]
                                if t["team"] == row["chain"]["team"]
                                and t["actor"] == carrier % 3
                                and 4 * t["first"] <= j < 4 * (t["last"] + 1)
                            ),
                            None,
                        )
                        if cmd:
                            caption = {
                                "shield": "Carrier protects the first touch",
                                "sidestep": "Carrier moves sideways",
                                "release": "Carrier prepares the pass",
                            }[cmd["action"]]
                draw.text((left + 13, 390), caption, font=font(20), fill=green if k else gold)
            frame.save(tmp / f"view-{i:03}.png")
        ffmpeg(
            "-framerate",
            "12.5",
            "-i",
            tmp / "view-%03d.png",
            "-c:v",
            "libx264",
            "-crf",
            17,
            "-pix_fmt",
            "yuv420p",
            movie,
        )
        length = len(list(tmp.glob("view-*.png"))) / 12.5
        gif = make_gif(movie, movie.with_suffix(".gif"), 0, length, 12.5)
    save(
        movie.with_suffix(".json"),
        evidence
        | dict(
            excerpt=gif,
            mp4_sha256=digest(movie),
            speed=1,
            native_sources={name: digest(a.pair / name / "match.mp4") for name in names},
        ),
    )


if __name__ == "__main__":
    main()
