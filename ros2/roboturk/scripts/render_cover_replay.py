"""Render paired native 5v5 match excerpts with the traced player's actual assignment."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageEnhance

from audit_five_roles import load_match
from render_waypoint_replay import font
from roboturk.five_roles import PENALTY_FRONT, PENALTY_HALF_WIDTH
from roboturk.timed_motion import inside

WIDTH = 510
CAMERA_Z = 16.5
BACKGROUND = "#10202b"
CLOSE_VIEW = (-4.96, 1.65, -1.0, -1.65)
FULL_FIELD = (-5.05, 3.2, 5.05, -3.2)


def panel(folder, game, states, robot, label, view, switch_tick=None):
    player = game["keys"].index(list(robot))
    robot_positions = states[:, 4:].reshape(len(states), len(game["keys"]), 3)[:, player, :2]
    files = sorted((folder / "frames").glob("*.jpg"))
    if not files:
        raise ValueError(f"No native match frames in {folder}")
    frames, trail = [], []
    large, small = font(20, True), font(15)
    left, top, right, bottom = view
    height = round(WIDTH * (top - bottom) / (right - left))
    radius = max(10, round(0.14 * WIDTH / (right - left)))
    diamond = max(5, radius // 2)
    for path in files:
        tick = 2 * int(path.stem)
        with Image.open(path) as source:
            w, h = source.size

            def native_pixel(position, z=0):
                scale = w / (2 * (CAMERA_Z - z) * math.tan(math.pi / 8))
                return w / 2 + position[0] * scale, h / 2 - position[1] * scale

            x0, y0 = native_pixel((left, top))
            x1, y1 = native_pixel((right, bottom))
            crop = tuple(map(round, (x0, y0, x1, y1)))
            factor = WIDTH / (crop[2] - crop[0])
            vertical_factor = height / (crop[3] - crop[1])
            scene = (
                source.convert("RGB").crop(crop).resize((WIDTH, height), Image.Resampling.LANCZOS)
            )
        scene = ImageEnhance.Brightness(scene).enhance(1.45)

        def pixel(position, z=0):
            x, y = native_pixel(position, z)
            return (x - crop[0]) * factor, (y - crop[1]) * vertical_factor

        trail.append(pixel(robot_positions[tick], 0.15))
        draw = ImageDraw.Draw(scene)
        for sign in (-1, 1):
            box = [
                (sign * 4.5, -PENALTY_HALF_WIDTH),
                (sign * PENALTY_FRONT, -PENALTY_HALF_WIDTH),
                (sign * PENALTY_FRONT, PENALTY_HALF_WIDTH),
                (sign * 4.5, PENALTY_HALF_WIDTH),
            ]
            for start, end in zip(box, box[1:]):
                ax, ay = pixel(start)
                bx, by = pixel(end)
                length = math.hypot(bx - ax, by - ay)
                for offset in range(0, math.ceil(length), 10):
                    u, v = offset / length, min(1, (offset + 5) / length)
                    draw.line(
                        (
                            ax + u * (bx - ax),
                            ay + u * (by - ay),
                            ax + v * (bx - ax),
                            ay + v * (by - ay),
                        ),
                        fill="#bed9d6",
                    )
        if len(trail) > 1:
            draw.line(trail, fill="#abdbff", width=2)
        x, y = trail[-1]
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), outline="white", width=2)
        meta = game["frames"][tick]
        motion = meta["motion"][player]
        caption = meta["roles"][player]
        if motion:
            tx, ty = pixel(motion["target"])
            target_ok = inside(motion["target"])
            color = "#ff806a" if not target_ok else "#9aeccc"
            draw.polygon(
                [(tx, ty - diamond), (tx + diamond, ty), (tx, ty + diamond), (tx - diamond, ty)],
                fill=color,
                outline="white",
            )
            if not target_ok:
                caption += " · Target overlaps clearance zone"
            elif (
                math.dist(robot_positions[tick], motion["target"]) < 0.03
                and motion["speed_mps"] < 0.08
            ):
                caption += " · In position"
            else:
                caption += f" · {motion['mode'].title()}"
        if switch_tick is not None and switch_tick <= tick < switch_tick + 15:
            caption = f"Target shifted to legal edge · {meta['roles'][player]}"

        canvas = Image.new("RGB", (WIDTH, height + 78), BACKGROUND)
        canvas.paste(scene, (0, 39))
        draw = ImageDraw.Draw(canvas)
        draw.text((11, 7), label, font=large, fill="white")
        draw.text((426, 10), f"{tick * game['dt']:.2f} s", font=small, fill="white")
        draw.text(
            (11, canvas.height - 29),
            f"{robot[0].title()} {robot[1]}  ·  {caption}",
            font=small,
            fill="white",
        )
        frames.append(canvas)
    return frames


def render(baseline, corrected, output, robot, view):
    old, a = load_match(baseline)
    new, b = load_match(corrected)
    cfg = json.loads((corrected / "config.json").read_text())
    switch = cfg["legal_off_ball_from_tick"]
    if old["seed"] != new["seed"] or old["keys"] != new["keys"] or old["dt"] != new["dt"]:
        raise ValueError("Recordings must use the same match start")
    if old["frames"][:switch] != new["frames"][:switch] or not np.array_equal(
        a[: switch + 1], b[: switch + 1]
    ):
        raise ValueError("Recordings diverged before the correction was enabled")
    if sorted(p.name for p in (baseline / "frames").glob("*.jpg")) != sorted(
        p.name for p in (corrected / "frames").glob("*.jpg")
    ):
        raise ValueError("Recordings must use the same frame window")
    left = panel(baseline, old, a, robot, "Released target assignment", view)
    right = panel(corrected, new, b, robot, "Targets outside both boxes", view, switch)
    frames = []
    for first, second in zip(left, right, strict=True):
        frame = Image.new("RGB", (2 * WIDTH + 3, first.height), "#acb6be")
        frame.paste(first)
        frame.paste(second, (WIDTH + 3, 0))
        frames.append(frame)
    sample = frames[::8]
    strip = Image.new("RGB", (frames[0].width, frames[0].height * len(sample)))
    for i, frame in enumerate(sample):
        strip.paste(frame, (0, i * frame.height))
    palette = strip.quantize(colors=224)
    frames = [frame.quantize(palette=palette, dither=Image.Dither.NONE) for frame in frames]
    frames[0].save(
        output, save_all=True, append_images=frames[1:], duration=80, loop=0, optimize=True
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("corrected", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--team", choices=("blue", "yellow"), default="blue")
    parser.add_argument("--robot", type=int, default=2)
    parser.add_argument("--full-field", action="store_true")
    args = parser.parse_args()
    view = FULL_FIELD if args.full_field else CLOSE_VIEW
    render(args.baseline, args.corrected, args.output, (args.team, args.robot), view)
