"""Render native Webots captures with role markers and the measured outcome."""

import argparse
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageEnhance

from compare_keeper_cover import PANELS
from compare_tackle_selector import read_rows
from render_waypoint_replay import font

WIDTH, HEIGHT = 510, 288


def panel(folder, row, arm, historical):
    result = row["outcomes"][arm]
    side_name = result["decisions"][0]["presser"]["team"]
    sign = row["initial"]["attack_sign"] * (1 if side_name == "blue" else -1)
    if row["zone"] == "keeper-timing":
        title = "Current cover" if arm == "keeper-cover" else "Timed interception"
    elif row["zone"] == "keeper-lane":
        title = "Current cover" if arm == "keeper-cover" else "Fieldward breakout"
    else:
        title = (
            "Archived selector"
            if historical
            else "Current selector"
            if arm == "selector"
            else "Presser + keeper"
        )
    scale = WIDTH / (2 * 3.25 * math.tan(math.pi / 8))
    early = (
        next(
            (
                decision["time"]
                for decision in result["decisions"]
                if any("own_goal" in risks for risks in decision["vetoes"].values())
            ),
            None,
        )
        if result.get("keeper_ready_ticks", 0)
        else None
    )
    frames = []
    # The zoom applies on the Webots step after the first sample.
    for sample in result["film"][1:]:
        t = sample["time"]
        with Image.open(folder / "frames" / arm / sample["file"]) as source:
            scene = source.convert("RGB").resize((WIDTH, HEIGHT), Image.Resampling.LANCZOS)
        scene = ImageEnhance.Brightness(scene).enhance(1.28)
        draw = ImageDraw.Draw(scene)
        for key, tag, color, radius in (
            ("presser", "P", "#fff2a4", 20),
            ("keeper", "K", "#9bf0e4", 20),
            ("ball", "", "#ffbb6c", 6),
        ):
            x, y = sample[key]
            x, y = WIDTH / 2 + (x + sign * 4.10) * scale, HEIGHT / 2 - y * scale
            draw.ellipse((x - radius, y - radius, x + radius, y + radius), outline=color, width=2)
            if tag:
                draw.text((x + 21, y - 10), tag, font=font(17, True), fill=color, stroke_width=2)
        board = Image.new("RGB", (WIDTH, HEIGHT + 76), "#13232d")
        board.paste(scene, (0, 38))
        text = ImageDraw.Draw(board)
        text.text((10, 7), title, font=font(21, True), fill="white")
        text.text((WIDTH - 78, 9), f"{t:.2f} s", font=font(17), fill="white")
        status = "Keeper parked in the opposite half" if historical else "P  Presser     K  Keeper"
        if early is not None and t >= early:
            status = "Keeper moves early after a goalward veto"
        if result["first_cover_s"] is not None and t >= result["first_cover_s"]:
            status = "Keeper handoff activated"
        goal = result["own_goal_s"]
        capture = result["capture_s"]
        if goal is not None and t >= goal:
            status = "Own goal"
        elif capture is not None and t >= capture and result["clean_retained"]:
            status = f"{result['captured_by'].title()} secures the ball"
        elif len(frames) == len(result["film"]) - 2 and goal is None:
            status = "Ball kept out"
        text.text((10, HEIGHT + 47), status, font=font(17), fill="white")
        frames.append(board)
    if not frames:
        raise ValueError("The trial needs native frames recorded with --frame-images")
    return frames


def render(folder, destination, historical=False):
    (row,) = read_rows(folder / "tackle-panel.json")
    arms = ("selector",) if historical else PANELS[row["zone"]]
    panels = [panel(folder, row, arm, historical) for arm in arms]
    frames = []
    for i in range(max(map(len, panels))):
        frame = Image.new("RGB", (len(arms) * (WIDTH + 3) - 3, HEIGHT + 76), "#516371")
        for n, source in enumerate(panels):
            frame.paste(source[min(i, len(source) - 1)], (n * (WIDTH + 3), 0))
        frames.append(frame)
    frames[0].save(
        destination,
        save_all=True,
        append_images=frames[1:],
        duration=[155] * (len(frames) - 1) + [750],
        loop=0,
        optimize=True,
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--historical", action="store_true")
    args = parser.parse_args()
    render(args.recording, args.output, args.historical)
