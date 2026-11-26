"""Annotate a clean tackle using native frames verified against a 5v5 benchmark trace."""

import argparse
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageEnhance

from compare_live_takeaways import load_match, summarize
from render_waypoint_replay import font

WIDTH, HEIGHT = 690, 390


def render(recording, benchmark, reach, output):
    game, states = load_match(benchmark)
    replay, replay_states = load_match(recording)
    end = len(replay_states)
    if replay["seed"] != game["seed"] or not np.array_equal(replay_states, states[:end]):
        raise ValueError("Native recording differs from the benchmark simulation")
    with (
        np.load(recording / "possession.npz") as a,
        np.load(benchmark / "possession.npz") as b,
    ):
        if any(not np.array_equal(a[k], b[k][: len(a[k])]) for k in ("rollers", "contacts")):
            raise ValueError("Recording differs in substep contact or possession")
    events = [
        e for e in summarize(benchmark, game)["transfers"] if e["clean"] and e["trained_tackle"]
    ]
    event = min(events, key=lambda e: abs(e["reach_s"] - reach))
    keys = game["keys"]
    actor, carrier = (keys.index(event[k]) for k in ("actor", "carrier"))

    def name(index):
        return f"{keys[index][0].title()} {keys[index][1]}"

    files = sorted((recording / "frames").glob("*.jpg"))
    ticks = [int(p.stem) * 2 for p in files]
    dt = game["dt"]
    if not ticks or not ticks[0] * dt < event["reach_s"] < event["confirmed_s"] < ticks[-1] * dt:
        raise ValueError("The video window must include the clean tackle")
    robot = states[:, 4:].reshape(len(states), len(keys), 3)
    group = robot[ticks][:, [actor, carrier], :2].reshape(-1, 2)
    center_x, center_y = (group.max(axis=0) + group.min(axis=0)) / 2
    span = max(3.7, np.ptp(group[:, 0]) + 1.2, (np.ptp(group[:, 1]) + 1.2) * WIDTH / HEIGHT)
    large, small = font(23, True), font(17)
    frames = []
    for path, tick in zip(files, ticks, strict=True):
        with Image.open(path) as source:
            width, height = source.size
            px_per_m = width / (2 * 16.5 * math.tan(math.pi / 8))
            length = span * px_per_m
            cx, cy = width / 2 + center_x * px_per_m, height / 2 - center_y * px_per_m
            tall = length * HEIGHT / WIDTH
            crop = tuple(
                map(round, (cx - length / 2, cy - tall / 2, cx + length / 2, cy + tall / 2))
            )
            scene = (
                source.convert("RGB").crop(crop).resize((WIDTH, HEIGHT), Image.Resampling.LANCZOS)
            )
        scene = ImageEnhance.Brightness(scene).enhance(1.35)
        draw = ImageDraw.Draw(scene)

        def display_point(x, y, z):
            scale = px_per_m * 16.5 / (16.5 - z)
            return (
                (width / 2 + x * scale - crop[0]) * WIDTH / (crop[2] - crop[0]),
                (height / 2 - y * scale - crop[1]) * HEIGHT / (crop[3] - crop[1]),
            )

        for index in (carrier, actor):
            color = "#ffe798" if keys[index][0] == "yellow" else "#acedff"
            x, y = display_point(*robot[tick, index, :2], 0.15)
            draw.ellipse((x - 22, y - 22, x + 22, y + 22), outline=color, width=3)
        x, y = display_point(*states[tick, :2], game["frames"][tick]["ball_z"])
        draw.ellipse((x - 5, y - 5, x + 5, y + 5), outline="#ffbc6d", width=2)
        canvas = Image.new("RGB", (WIDTH, HEIGHT + 81), "#10202b")
        canvas.paste(scene, (0, 43))
        draw = ImageDraw.Draw(canvas)
        now = tick * dt
        draw.text((12, 8), f"{name(actor)} challenges {name(carrier)}", font=large, fill="white")
        draw.text((WIDTH - 89, 11), f"{now:.2f} s", font=small, fill="white")
        status = "Presser closes on the ball"
        if now >= event["confirmed_s"]:
            status = "Clean takeaway confirmed"
        elif now >= event["reach_s"]:
            status = "Presser reaches the ball"
        draw.text((12, HEIGHT + 53), status, font=small, fill="#acedff")
        frames.append(canvas)
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=[120] * (len(frames) - 1) + [560],
        loop=0,
        optimize=True,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("benchmark", type=Path)
    parser.add_argument("reach", type=float)
    parser.add_argument("output", type=Path)
    render(**vars(parser.parse_args()))
