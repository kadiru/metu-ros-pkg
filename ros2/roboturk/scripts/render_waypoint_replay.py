"""Annotate two matched native Webots outlet recordings with the runner and its waypoint."""

import argparse
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageEnhance, ImageFont

WIDTH = 510
CAPTIONS = {
    "direct": "Direct route",
    "detour": "Taking a fixed detour",
    "brake": "Braking",
    "errt": "Choosing an ERRT waypoint",
    "escape": "Choosing an outward waypoint",
    "hold": "Following the waypoint",
    "shortcut": "Releasing waypoint for direct route",
}


def read_json(path):
    return json.loads(path.read_text())


def font(size, bold=False):
    candidates = (
        ("DejaVuSans-Bold.ttf", "Arial Bold.ttf") if bold else ("DejaVuSans.ttf", "Arial.ttf")
    )
    for name in candidates:
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def panel(folder, label):
    rows = read_json(folder / "crowded.json")
    seed = read_json(folder / "config.json").get("film_seed")
    if seed is not None:
        rows = [r for r in rows if r["seed"] == seed]
    (row,) = rows
    trace = read_json(folder / "runner-trace.json")
    start = next(r for r in row["initial"]["robots"] if r["key"] == row["runner"])
    center = ((start["x"] + row["target"][0]) / 2, (start["y"] + row["target"][1]) / 2)
    files = sorted((folder / "frames").glob("*.jpg"))
    if not files or len(files) != (len(trace) + 1) // 2:
        raise ValueError(f"Incomplete native frames or runner trace in {folder}")
    large, small = font(20, True), font(15)
    frames, trail = [], []
    team = row["runner"]["team"].title()

    for index, (file, step) in enumerate(zip(files, trace[::2], strict=True)):
        with Image.open(file) as source:
            height = round(WIDTH * source.height / source.width)
            scene = ImageEnhance.Brightness(
                source.convert("RGB").resize((WIDTH, height), Image.Resampling.LANCZOS)
            ).enhance(1.5)

        # The fixture uses a top-down camera at 6.4 m with a 45-degree horizontal FOV.
        def pixel(pos, z):
            scale = WIDTH / (2 * (6.4 - z) * math.tan(math.pi / 8))
            return WIDTH / 2 + (pos[0] - center[0]) * scale, height / 2 - (
                pos[1] - center[1]
            ) * scale

        trail.append(pixel(step["xy"], 0.15))
        draw = ImageDraw.Draw(scene)
        if len(trail) > 1:
            draw.line(trail, fill="#dfebf2", width=2)
        x, y = trail[-1]
        draw.ellipse((x - 12, y - 12, x + 12, y + 12), outline="white", width=2)
        if step["waypoint"] is not None:
            x, y = pixel(step["waypoint"], 0)
            draw.polygon(
                [(x, y - 6), (x + 6, y), (x, y + 6), (x - 6, y)], fill="#d755ab", outline="white"
            )
        # Webots windows can differ by a few native pixels. Keep the common
        # horizontal field of view and center the scene in a 16:9 viewport.
        display_height = round(WIDTH * 9 / 16)
        top = (scene.height - display_height) // 2
        scene = scene.crop((0, top, WIDTH, top + display_height))
        canvas = Image.new("RGB", (WIDTH, display_height + 77), "#10202b")
        canvas.paste(scene, (0, 39))
        draw = ImageDraw.Draw(canvas)
        draw.text((11, 7), label, font=large, fill="white")
        draw.text((442, 11), f"{step['t']:.1f} s", font=small, fill="white")
        outcome = row["arrival_s"]
        caption = CAPTIONS[step["mode"]]
        recent = trace[max(0, 2 * index - 12) : 2 * index + 1]
        if step["mode"] in ("direct", "shortcut") and any(s["mode"] == "shortcut" for s in recent):
            caption = CAPTIONS["shortcut"]
        if outcome is not None and step["t"] >= outcome:
            caption = f"Reached outlet at {outcome:.2f} s"
        elif outcome is None and file == files[-1]:
            caption = f"Timed out at {row['limit_s']:g} s"
        draw.text((11, canvas.height - 29), f"{team}   ·   {caption}", font=small, fill="white")
        frames.append(canvas)
    return row, frames


def render(previous, committed, output, previous_label="Released ERRT", updated_label=None):
    a, left = panel(previous, previous_label)
    mode = read_json(committed / "config.json")["commit_waypoints"]
    label = "Selective commitment" if mode == "on-conflict" else "Immediate commitment"
    b, right = panel(committed, updated_label or label)
    if any(a[k] != b[k] for k in ("seed", "initial", "runner", "target", "limit_s")):
        raise ValueError("Recordings must use identical starts, runners, targets and deadlines")
    frames = []
    for lframe, rframe in zip(left, right, strict=True):
        frame = Image.new("RGB", (WIDTH * 2 + 3, lframe.height), "#acb6be")
        frame.paste(lframe)
        frame.paste(rframe, (WIDTH + 3, 0))
        frames.append(frame)
    strip = Image.new("RGB", (WIDTH * 2 + 3, len(frames[::8]) * frames[0].height))
    for i, frame in enumerate(frames[::8]):
        strip.paste(frame, (0, i * frame.height))
    palette = strip.quantize(colors=224)
    frames = [f.quantize(palette=palette, dither=Image.Dither.NONE) for f in frames]
    frames[0].save(
        output, save_all=True, append_images=frames[1:], duration=80, loop=0, optimize=True
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("previous", type=Path)
    p.add_argument("committed", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--previous-label", default="Released ERRT")
    p.add_argument("--updated-label")
    args = p.parse_args()
    render(args.previous, args.committed, args.output, args.previous_label, args.updated_label)
