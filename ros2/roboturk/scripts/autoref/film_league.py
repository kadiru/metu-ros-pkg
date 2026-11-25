"""Caption an uncut Webots evaluation game using the recorded referee broadcast."""

import argparse
from bisect import bisect_right
from functools import lru_cache
import math
from pathlib import Path
import subprocess
from PIL import Image, ImageDraw

from audit_league import audit
from film import font
from film_open_motion import BG, BLUE, WHITE, YELLOW, encode, pitch, recording

STAGES = {
    "": "Waiting for referee",
    "NORMAL_FIRST_HALF_PRE": "First kickoff",
    "NORMAL_FIRST_HALF": "First half",
    "NORMAL_HALF_TIME": "Half-time",
    "NORMAL_SECOND_HALF_PRE": "Second kickoff",
    "NORMAL_SECOND_HALF": "Second half",
    "POST_GAME": "Full time",
}
FONTS = lru_cache()(font)


def show_reported_pair(rec, output):
    log, states, report, *_ = rec
    case = next(
        (
            c
            for c in report["observed_crashes"]
            if c["nearest_centers_previous_second_m"] > 1 and c["since_formation_s"] < 2.5
        ),
        None,
    )
    if case is None:
        return
    tick = case["tick"]
    pair = [log["keys"].index([team, case[team]]) for team in ("blue", "yellow")]
    label = f"Blue {case['blue']} and Yellow {case['yellow']}"
    frames = []
    for t in range(max(0, (tick // 2 - 24) * 2), min(len(states), tick + 29), 2):
        panel = Image.new("RGB", (640, 520), BG)
        panel.paste(pitch(rec, t, (0, 0), 10, (640, 394), pair), (0, 74))
        draw = ImageDraw.Draw(panel)
        draw.text((16, 10), "A crash call after a kickoff reset", font=FONTS(24), fill=WHITE)
        xy = [states[t, 4 + 3 * k : 6 + 3 * k] for k in pair]
        draw.text((16, 45), f"{label}     {math.dist(*xy):.2f} m apart", font=FONTS(18), fill=WHITE)
        for (x, y), color in zip(xy, (BLUE, YELLOW), strict=True):
            cx = 320 + 64 * x * 16.5 / (16.5 - 0.15)
            cy = 74 + 197 - 64 * y * 16.5 / (16.5 - 0.15)
            draw.ellipse((cx - 10, cy - 10, cx + 10, cy + 10), outline=color, width=2)
        caption = log["frames"][t]["ssl_command"].replace("_", " ").title()
        if t >= tick:
            caption = f"TIGERs reported a crash between {label}"
        draw.text((16, 481), caption, font=FONTS(18), fill=WHITE)
        frames.append(panel)
    frames[0].save(
        output / "kickoff-call.gif", save_all=True, append_images=frames[1:], duration=160, loop=0
    )


def film(folder, output, *, halftime_clip=False):
    output.mkdir(parents=True, exist_ok=True)
    report, _, _ = audit(folder, require_native_goals=not halftime_clip)
    rec = recording(folder / "match", report)
    show_reported_pair(rec, output)
    log = rec[0]
    frames = log["frames"]
    marks = []
    for e in log["events"]:
        if e["event"] in ("pass", "shot", "restart_pass"):
            team, num = log["keys"][e["actor"]]
            if e["event"] == "shot":
                msg = f"{team.title()} {num} shoots"
            else:
                msg = (
                    f"{team.title()} {num} passes to {team.title()} {log['keys'][e['receiver']][1]}"
                )
            marks.append((e["tick"], msg))
    for p in report["completed"]:
        team, num = p["receiver"]
        marks.append((p["receive"], f"{team.title()} {num} receives"))
    for g in report["goals"]:
        if g["crossing_tick"] is not None:
            marks.append((g["crossing_tick"], "Ball crosses the goal line"))
        marks.append((g["score_tick"], f"Official goal for {g['team'].title()}"))
    marks.sort()
    ticks = [m[0] for m in marks]

    def frame(t):
        f = frames[t]
        base = Image.new("RGB", (800, 640), BG)
        draw = ImageDraw.Draw(base)
        draw.text((22, 12), "RoboTurk   5v5", font=FONTS(30), fill=WHITE)
        draw.text(
            (780, 16), "Webots  /  SSL Game Controller", anchor="ra", font=FONTS(19), fill="#b4c6cb"
        )
        period = STAGES[f["stage"]]
        clock = f["stage_seconds"]
        if clock is not None and f["stage"] in ("NORMAL_FIRST_HALF", "NORMAL_SECOND_HALF"):
            minutes, seconds = divmod(max(0, math.ceil(clock)), 60)
            period += f"    {minutes}:{seconds:02d}"
        draw.text((24, 59), period, font=FONTS(23), fill=WHITE)
        draw.text((600, 57), f"Blue   {f['score'][0]}", anchor="ra", font=FONTS(24), fill=BLUE)
        draw.text((776, 57), f"Yellow   {f['score'][1]}", anchor="ra", font=FONTS(24), fill=YELLOW)
        base.paste(pitch(rec, t, (0, 0), 10, (800, 492), range(10)), (0, 96))
        goals = [("BLUE", BLUE), ("YELLOW", YELLOW)]
        if f["attack_sign"] < 0:
            goals.reverse()
        for x, (team, color) in zip((39, 761), goals, strict=True):
            draw.rounded_rectangle((x - 32, 274, x + 32, 297), radius=4, fill=BG)
            draw.text((x, 286), team, anchor="mm", font=FONTS(14), fill=color)
        command = (f["ssl_command"] or "Awaiting broadcast").replace("_", " ").title()
        i = bisect_right(ticks, t) - 1
        msg = ""
        if i >= 0 and t - ticks[i] <= 30:
            msg = marks[i][1]
        draw.text((22, 602), command, font=FONTS(21), fill=WHITE)
        draw.text((777, 604), msg, anchor="ra", font=FONTS(19), fill="#bbd9c6")
        return base

    movie = output / "official-match.mp4"
    fps = 1 / (2 * log["dt"])

    # The final POST_GAME packet may be on an odd tick, between camera samples.
    def sampled():
        for t in range(0, len(frames), 2):
            yield frame(t)
        if (len(frames) - 1) % 2:
            yield frame(len(frames) - 1)

    encode(movie, sampled(), fps)
    if halftime_clip:
        half = next(
            i
            for i, f in enumerate(frames)
            if f["stage"] == "NORMAL_SECOND_HALF_PRE"
            and f["ssl_command"].startswith("PREPARE_KICKOFF")
        )
        release = next(
            i for i in range(half, len(frames)) if frames[i]["ssl_command"] == "NORMAL_START"
        )
        start = max(0, half * log["dt"] - 0.4)
        end = min(len(frames) * log["dt"], release * log["dt"] + 1.2)
    elif report["goals"]:
        g = report["goals"][0]
        start = max(0, g["crossing_tick"] * log["dt"] - 5.0)
        end = min(len(frames) * log["dt"], g["score_tick"] * log["dt"] + 2.4)
    else:
        start = (
            next(i for i, f in enumerate(frames) if f["stage"] == "NORMAL_HALF_TIME") * log["dt"]
            - 3
        )
        end = start + 10
    subprocess.run(
        [
            "ffmpeg",
            "-v",
            "error",
            "-y",
            "-ss",
            str(start),
            "-i",
            str(movie),
            "-t",
            str(end - start),
            "-vf",
            "fps=12.5,scale=640:-2,split[a][b];[a]palettegen[p];[b][p]paletteuse=dither=bayer",
            "-loop",
            "0",
            str(output / ("official-kickoff.gif" if halftime_clip else "match-highlight.gif")),
        ],
        check=True,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("match", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument(
        "--halftime-clip",
        action="store_true",
        help="Extract a real-time GIF from the half-time award through release",
    )
    a = p.parse_args()
    film(a.match, a.output, halftime_clip=a.halftime_clip)
