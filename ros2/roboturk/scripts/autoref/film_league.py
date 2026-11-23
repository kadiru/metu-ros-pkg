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


def film(folder, output):
    output.mkdir(parents=True, exist_ok=True)
    report, _, _ = audit(folder)
    rec = recording(folder / "match", report)
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
            period += f"    0:{max(0, math.ceil(clock)):02d}"
        draw.text((24, 59), period, font=FONTS(23), fill=WHITE)
        draw.text((600, 57), f"Blue   {f['score'][0]}", anchor="ra", font=FONTS(24), fill=BLUE)
        draw.text((776, 57), f"Yellow   {f['score'][1]}", anchor="ra", font=FONTS(24), fill=YELLOW)
        base.paste(pitch(rec, t, (0, 0), 10, (800, 492), range(10)), (0, 96))
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
    if report["goals"]:
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
            str(output / "match-highlight.gif"),
        ],
        check=True,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("match", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    film(a.match, a.output)
