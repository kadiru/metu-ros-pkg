"""Draw attacking-frame ball paths from a recovery-loss audit (requires Pillow)."""

import argparse
import json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont


def main():
    p = argparse.ArgumentParser()
    p.add_argument("report", type=Path)
    p.add_argument("panel", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    report = json.loads(a.report.read_text())
    import numpy as np

    bg = "#f5f5ef"
    ink = "#163127"
    muted = "#47594f"
    turf = "#e4eee4"
    im = Image.new("RGB", (1240, 474), bg)
    d = ImageDraw.Draw(im)

    def font(size, bold=False):
        for name in (
            ("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"),
            (
                "/System/Library/Fonts/Supplemental/Arial Bold.ttf"
                if bold
                else "/System/Library/Fonts/Supplemental/Arial.ttf"
            ),
        ):
            try:
                return ImageFont.truetype(name, size)
            except OSError:
                continue
        return ImageFont.load_default(size=size)

    d.text((25, 14), "Where the opening recovery broke down", font=font(30, True), fill=ink)
    d.text(
        (26, 58),
        "Both teams attack to the right. Circles mark recovery. Crosses mark an opponent touch or the ball leaving the field.",
        font=font(19),
        fill=muted,
    )
    first = json.loads((a.panel / "launch.json").read_text())["first"]
    for j, (label, suffix, color) in enumerate(
        [
            ("Before a kick", "before first kick", "#8d462f"),
            ("After a pass", "after pass", "#176c70"),
            ("After a shot", "after shot", "#66538a"),
        ]
    ):
        left = 25 + j * 407
        top = 150
        scale = 41

        def point(x, y):
            return left + (4.5 + x) * scale, top + (3 - y) * scale

        rows = [g for g in report["games"] if g["outcome"].endswith(suffix)]
        d.text((left, 105), f"{label}  ·  {len(rows)}", font=font(22, True), fill=ink)
        d.rectangle([point(-4.5, 3), point(4.5, -3)], fill=turf, outline="#8ca790", width=2)
        d.line([point(0, 3), point(0, -3)], fill="#9cb19e", width=1)
        d.ellipse([point(-0.5, 0.5), point(0.5, -0.5)], outline="#9cb19e")
        d.line([point(4.5, 0.48), point(4.5, -0.48)], fill=ink, width=5)
        for g in rows:
            direction = 1 if (g["seed"] % 2 == 0) == (g["team"] == "blue") else -1
            start = round(g["recovery_s"] * 100)
            end = start + round(g["elapsed_s"] * 100)
            with np.load(a.panel / f"trace-{g['seed'] - first:03}.npz") as trace:
                xy = trace["states"][start : end + 1, :2] * direction
            pts = [point(x, y) for x, y in xy[::4]] + [point(*xy[-1])]
            d.line(pts, fill=color, width=2)
            x, y = pts[0]
            d.ellipse([x - 4, y - 4, x + 4, y + 4], fill=bg, outline=color, width=2)
            x, y = pts[-1]
            d.line((x - 4, y - 4, x + 4, y + 4), fill=color, width=3)
            d.line((x - 4, y + 4, x + 4, y - 4), fill=color, width=3)
    counts = report["counts"]
    d.text(
        (26, 423),
        f"Previous team, {len(report['games'])} held-out contests. {counts.get('goal', 0)} goals and {counts.get('no exclusive recovery', 0)} contests without a verified recovery.",
        font=font(19),
        fill=muted,
    )
    im.save(a.output)


if __name__ == "__main__":
    main()
