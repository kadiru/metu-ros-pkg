"""Render native Webots captures with role markers and the measured outcome."""

import argparse
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageEnhance
from roboturk import field
from roboturk.model import wrap

from compare_keeper_cover import PANELS
from compare_tackle_selector import read_rows
from render_waypoint_replay import font

WIDTH, HEIGHT = 510, 288
TEAM_COLOR = {"blue": "#83bcff", "yellow": "#ffdd62"}


def pitch_point(x, y, sign, scale):
    return WIDTH / 2 + (x + sign * 4.10) * scale, HEIGHT / 2 - y * scale


def mark_goal(draw, team, sign, scale):
    color = TEAM_COLOR[team]
    line = -sign * field.LENGTH / 2
    back = line - sign * field.GOAL_DEPTH
    half = field.GOAL_WIDTH / 2
    frame = [
        pitch_point(x, y, sign, scale)
        for x, y in (
            (line, half),
            (back, half),
            (back, -half),
            (line, -half),
        )
    ]
    draw.line(frame, fill=color, width=4)
    label = f"{team.upper()} GOAL"
    label_font = font(15, True)
    width = math.ceil(draw.textlength(label, font=label_font))
    center = (frame[0][0] + frame[1][0]) / 2
    left = int(center - width / 2 - 7)
    draw.rounded_rectangle((left, 8, left + width + 14, 31), radius=6, fill=color)
    draw.text((left + 7, 10), label, font=label_font, fill="#12202d")
    draw.line(((center, 31), (center, frame[0][1])), fill=color, width=2)


def panel(folder, row, arm, historical):
    result = row["outcomes"][arm]
    side_name = result["decisions"][0]["presser"]["team"]
    attacking = "blue" if side_name == "yellow" else "yellow"
    sign = row["initial"]["attack_sign"] * (1 if side_name == "blue" else -1)
    if row["zone"] == "keeper-timing":
        title = "Current cover" if arm == "keeper-cover" else "Timed interception"
    elif row["zone"] == "keeper-lane":
        title = "Current cover" if arm == "keeper-cover" else "Fieldward breakout"
    elif row["zone"] == "keeper-pivot":
        title = "Fieldward sweep" if arm == "keeper-lane" else "Keeper pivot"
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
        mark_goal(draw, side_name, sign, scale)
        for key, tag, color, radius in (
            ("presser", "P", TEAM_COLOR[side_name], 20),
            ("keeper", "K", TEAM_COLOR[side_name], 20),
            ("ball", "", "#ffbb6c", 6),
        ):
            x, y = pitch_point(*sample[key], sign, scale)
            draw.ellipse((x - radius, y - radius, x + radius, y + radius), outline=color, width=2)
            if tag:
                draw.text((x + 21, y - 10), tag, font=font(17, True), fill=color, stroke_width=2)
        board = Image.new("RGB", (WIDTH, HEIGHT + 100), "#13232d")
        board.paste(scene, (0, 62))
        text = ImageDraw.Draw(board)
        text.text((10, 7), title, font=font(21, True), fill="white")
        text.text((WIDTH - 78, 9), f"{t:.2f} s", font=font(17), fill="white")
        team_font = font(15, True)
        attack_line = f"{attacking.upper()} ATTACKS"
        text.text((10, 38), attack_line, font=team_font, fill=TEAM_COLOR[attacking])
        text.text(
            (22 + text.textlength(attack_line, font=team_font), 38),
            f"{side_name.upper()} DEFENDS",
            font=team_font,
            fill=TEAM_COLOR[side_name],
        )
        status = "Keeper parked in the opposite half" if historical else "P  Presser     K  Keeper"
        if early is not None and t >= early:
            status = "Keeper moves early after a goalward veto"
        if result["first_cover_s"] is not None and t >= result["first_cover_s"]:
            status = "Keeper handoff activated"
        turns = [
            abs(math.degrees(wrap(p["yaw"] - p["start_yaw"])))
            for p in result.get("pivot_samples", ())
            if p["time"] <= t
        ]
        if turns and max(turns) >= 1:
            status = f"Keeper pivots {max(turns):.0f}° fieldward"
        goal = result["own_goal_s"]
        capture = result["capture_s"]
        if goal is not None and t >= goal:
            status = f"Goal conceded by {side_name.title()}"
        elif capture is not None and t >= capture and result["clean_retained"]:
            status = f"{result['captured_by'].title()} secures the ball"
        elif len(frames) == len(result["film"]) - 2 and goal is None:
            status = "Ball kept out"
        text.text((10, HEIGHT + 71), status, font=font(17), fill="white")
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
        frame = Image.new("RGB", (len(arms) * (WIDTH + 3) - 3, panels[0][0].height), "#516371")
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
