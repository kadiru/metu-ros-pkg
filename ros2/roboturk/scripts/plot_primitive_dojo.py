#!/usr/bin/env python3
"""Draw fixed-probe learning curves and held-out counts from a native report."""

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


TASK_NAMES = {
    "arrival": "Arrival pose and speed",
    "intercept": "Trap or body redirect",
    "pass": "Pass to feet or lead",
    "receive_race": "Contested reception",
}
INK, MUTED, GRID = "#163252", "#516982", "#d9e3ee"
FIT, BASELINE = "#067b73", "#62758d"


def font(size):
    return ImageFont.load_default(size=size)


def count(stats):
    return f"{stats['successes']}/{stats['trials']}"


def panel(draw, origin, task, data):
    x, y = origin
    draw.rounded_rectangle((x, y, x + 518, y + 294), radius=17, fill="white", outline=GRID)
    draw.text((x + 23, y + 18), TASK_NAMES[task], font=font(24), fill=INK)
    stages = data["learning"]
    plot_x, plot_y, plot_width, plot_height = x + 67, y + 66, 416, 132
    for percent in (0, 25, 50, 75, 100):
        yy = plot_y + (1 - percent / 100) * plot_height
        draw.line((plot_x, yy, plot_x + plot_width, yy), fill=GRID, width=1)
        draw.text((plot_x - 10, yy), f"{percent}%", font=font(13), fill=MUTED, anchor="rm")
    first, last = stages[0]["rollouts"], stages[-1]["rollouts"]
    px = [plot_x + (s["rollouts"] - first) * plot_width / max(1, last - first) for s in stages]
    for point, stage in zip(px, stages, strict=True):
        draw.text(
            (point, plot_y + plot_height + 10),
            str(stage["rollouts"]),
            font=font(14),
            fill=MUTED,
            anchor="mt",
        )
    for arm, color in (("baseline", BASELINE), ("fitted", FIT)):
        values = [item["probe"][arm] for item in stages]
        py = [plot_y + (1 - r["successes"] / r["trials"]) * plot_height for r in values]
        draw.line(list(zip(px, py, strict=True)), fill=color, width=3)
        for xx, yy in zip(px, py, strict=True):
            draw.ellipse((xx - 4, yy - 4, xx + 4, yy + 4), fill=color)
    draw.text((x + 276, y + 243), "Training episodes", font=font(14), fill=MUTED, anchor="mm")
    tests = data["qualification"]
    draw.text((x + 23, y + 271), "Held out", font=font(16), fill=INK, anchor="lm")
    draw.text(
        (x + 124, y + 271), count(tests["baseline"]), font=font(18), fill=BASELINE, anchor="lm"
    )
    draw.text((x + 204, y + 271), count(tests["fitted"]), font=font(18), fill=FIT, anchor="lm")
    if "post_release" in tests:
        label = f"Post-release {count(tests['post_release'])}"
        draw.text((x + 493, y + 271), label, font=font(15), fill=MUTED, anchor="rm")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("report", type=Path)
    parser.add_argument("image", type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text())
    canvas = Image.new("RGB", (1100, 766), "#f4f8fc")
    draw = ImageDraw.Draw(canvas)
    draw.text((25, 24), "RoboTurk’s Webots training drills", font=font(36), fill=INK)
    draw.text(
        (27, 78),
        "Success rate on a fixed probe as each controller sees more training starts",
        font=font(18),
        fill=MUTED,
    )
    for idx, (arm, color) in enumerate((("Baseline", BASELINE), ("Fitted controller", FIT))):
        x = 27 + idx * 129
        draw.line((x, 117, x + 19, 117), fill=color, width=4)
        draw.text((x + 25, 117), arm, font=font(16), fill=color, anchor="lm")
    for idx, (task, data) in enumerate(report["tasks"].items()):
        panel(draw, (23 + 536 * (idx % 2), 145 + 309 * (idx // 2)), task, data)
    canvas.save(args.image)


if __name__ == "__main__":
    main()
