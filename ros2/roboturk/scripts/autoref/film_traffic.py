"""Compare camera frames of the old and trained placers on the same touchline exit."""

import argparse
import json
import math
from pathlib import Path
from PIL import Image, ImageDraw
import numpy as np

from film import font
from verify import load_events


def clip(path):
    (row,) = json.loads((path / "audit.json").read_text())["trials"]
    log = json.loads((path / "match/film-trace.json").read_text())
    files = sorted((path / "match/frames").glob("*.jpg"))
    keys = [r[:2] for r in row["opening_robots"]]
    assert len(log) == len(files) and (row["team"], row["placer"]) == ("blue", 4)
    first = next(t for t, c, _ in row["commands"] if c == "BALL_PLACEMENT_BLUE")
    calls = [
        stamp - row["vision_start"] + log[0]["time"]
        for stamp, kind, _ in load_events(path / "tigers.log")
        if kind in ("bot_crash_unique", "bot_crash_drawn")
    ]
    return row, log, files, first, calls, keys.index(["blue", 4]), keys.index(["yellow", 2])


def compare(base, new, dest):
    panels = [clip(base), clip(new)]
    assert abs(panels[0][3] - panels[1][3]) < 0.001
    a, b = panels
    before = [(p, q) for p, q in zip(a[1], b[1]) if p["time"] < a[3]]
    initial_delta = max(
        math.dist(i, j)
        for p, q in before
        for i, j in zip([p["ball"], *p["robots"]], [q["ball"], *q["robots"]], strict=True)
    )
    assert initial_delta < 1e-4, initial_delta
    assert not b[4] and b[0]["accepted"] and b[0].get("received_seconds")
    titles = ["Released placer", "Trained approach"]
    frames = []
    begin = int(round((a[3] - a[1][0]["time"] - 0.48) / 0.08))
    for n in range(begin, begin + 56):
        canvas = Image.new("RGB", (900, 414), "#10212b")
        d = ImageDraw.Draw(canvas)
        for k, (row, trace, files, at, calls, placer, blocker) in enumerate(panels):
            idx = min(n, len(files) - 1)
            f = trace[idx]
            image = Image.open(files[idx]).convert("RGB")
            w, h = image.size
            line = np.array(image)[h // 3].astype(float)
            green = np.flatnonzero(
                (line[:, 1] > 1.5 * line[:, 0]) & (line[:, 1] > 1.35 * line[:, 2])
            )
            px = (green[-1] - green[0] + 1) / 10
            left, top = round((w - 1) / 2 - 0.38 * px), round((h - 1) / 2 - 3.50 * px)
            sw, sh = round(2.72 * px), round(2.04 * px)
            panel = image.crop((left, top, left + sw, top + sh)).resize(
                (448, 336), Image.Resampling.LANCZOS
            )
            draw = ImageDraw.Draw(panel)

            def point(p, z):
                scale = 16.5 / (16.5 - z)
                return (
                    ((w - 1) / 2 + p[0] * px * scale - left) * 448 / sw,
                    ((h - 1) / 2 - p[1] * px * scale - top) * 336 / sh,
                )

            for bot, color in ((placer, "#afd5ff"), (blocker, "#ffe394")):
                trail = [point(x["robots"][bot], 0.15) for x in trace[max(0, idx - 8) : idx + 1]]
                if len(trail) > 1:
                    draw.line(trail, fill=color, width=2)
                x, y = trail[-1]
                draw.text(
                    (x + 16, y - 15),
                    "4" if bot == placer else "2",
                    font=font(15),
                    fill=color,
                    stroke_width=1,
                    stroke_fill="black",
                )
            x, y = point(f["ball"], 0.0215)
            draw.ellipse((x - 6, y - 6, x + 6, y + 6), outline="#ffb653", width=2)
            left_panel = k * 452
            canvas.paste(panel, (left_panel, 41))
            d.text((left_panel + 12, 8), titles[k], font=font(21), fill="white")
            separation = math.dist(f["robots"][placer], f["robots"][blocker])
            label = f"Center separation   {100 * separation:.0f} cm"
            if calls and f["time"] >= calls[0] - 0.08:
                label = "TIGERs reported a collision"
            d.text((left_panel + 12, 384), label, font=font(17), fill="#f4dea9")
        frames.append(canvas)
    frames[0].save(
        dest, save_all=True, append_images=frames[1:], duration=80, loop=0, optimize=True
    )
    return dict(frames=len(frames), max_preplacement_pose_delta_m=initial_delta)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("baseline", type=Path)
    p.add_argument("candidate", type=Path)
    p.add_argument("gif", type=Path)
    a = p.parse_args()
    print(compare(a.baseline, a.candidate, a.gif))
