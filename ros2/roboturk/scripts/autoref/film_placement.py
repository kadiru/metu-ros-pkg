"""Encode and annotate a continuous native Webots restart from audited fixture output."""

import argparse
import json
import math
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw
from film import font


def film(run, dest):
    audit = json.loads((run / "audit.json").read_text())
    (row,) = audit["trials"]
    trace = json.loads((run / "match/film-trace.json").read_text())
    frames = sorted((run / "match/frames").glob("*.jpg"))
    first, second = row["kicks"]
    boundary, accepted = row["controller_events"]
    team = row["team"]
    if not (
        row["accepted"]
        and audit["interference"] == 0
        and len(trace) == len(frames)
        and boundary["type"] == "BALL_LEFT_FIELD_TOUCH_LINE"
        and boundary["ballLeftFieldTouchLine"]["byTeam"] == first["team"].upper()
        and accepted["type"] == "PLACEMENT_SUCCEEDED"
        and first["team"] != team
        and second["team"] == team
        and second["id"] == row["placer"]
        and row["received_seconds"] > second["seconds"]
        and not row["flight_contacts"]
        and math.dist(second["ball"], row["received_ball"]) > 1
    ):
        raise ValueError("Incomplete referee-to-reception sequence")
    width, height = 860, 426
    title, body, small = font(23), font(18), font(15)
    # Orthographic-looking overhead perspective. Calibrate against the actual
    # visible floor width (10 metres) at a scanline clear of the on-field HUD.
    a = np.asarray(Image.open(frames[0])).astype(int)
    h, w = a.shape[:2]
    r, g, b = a[h // 3].T
    green = np.flatnonzero((g > 1.5 * r) & (g > 1.35 * b))
    px = (green[-1] - green[0] + 1) / 10
    cx, cy = (w - 1) / 2, (h - 1) / 2
    left = int(cx - 2.68 * px)
    top = int(cy - 3.52 * px)
    crop_width = round(5.4 * px)
    crop_height = round(crop_width * height / width)

    def point(pos, z=0):
        f = 16.5 / (16.5 - z)
        return (
            (cx + f * pos[0] * px - left) * width / crop_width,
            (cy - f * pos[1] * px - top) * height / crop_height + 52,
        )

    dest.mkdir(parents=True, exist_ok=True)
    movie = dest / "touchline-restart.mp4"
    total_height = height + 108
    writer = [
        "ffmpeg",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{width}x{total_height}",
        "-r",
        "25/2",
        "-i",
        "-",
        "-c:v",
        "libx264",
        "-crf",
        "18",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(movie),
    ]
    keys = [(r[0], r[1]) for r in row["opening_robots"]]
    with subprocess.Popen(writer, stdin=subprocess.PIPE) as enc:
        for path, f in zip(frames, trace, strict=True):
            strip = (
                Image.open(path)
                .convert("RGB")
                .crop((left, top, left + crop_width, top + crop_height))
            )
            canvas = Image.new("RGB", (width, total_height), "#10212b")
            canvas.paste(strip.resize((width, height), Image.Resampling.LANCZOS), (0, 52))
            d = ImageDraw.Draw(canvas)
            cmd = f["command"]
            phase = f["phase"]
            label = "Yellow 1 aims a pass toward Yellow 2 on the wing."
            if cmd == "STOP" and phase == "opening":
                label = "TIGERs calls the touchline exit off Yellow 1."
            elif phase == "capture":
                label = (
                    f"Blue {row['placer']} goes to collect. Yellow clears the placement corridor."
                )
            elif phase == "carry":
                label = "The placer dribbles to the cross, the Game Controller’s designated point."
            elif phase == "retreat":
                label = "The placer withdraws while TIGERs checks ball motion and clearance."
            if cmd == "STOP" and phase == "retreat":
                label = "The Game Controller accepts the placement."
            elif cmd.startswith("DIRECT_FREE"):
                label = (
                    f"Blue {row['placer']} takes the awarded kick. Blue {row['receiver']} receives."
                )
            bp = point(f["ball"], 0.022)
            if phase != "opening":
                tx, ty = point(row["target"])
                d.line((tx - 11, ty, tx + 11, ty), fill="#70e8dc", width=2)
                d.line((tx, ty - 11, tx, ty + 11), fill="#70e8dc", width=2)
            x, y = bp
            d.ellipse((x - 7, y - 7, x + 7, y + 7), outline="#ffb556", width=2)
            for key, location in zip(keys, f["robots"], strict=True):
                x, y = point(location, 0.15)
                if 8 < x < width - 32 and 67 < y < height + 35:
                    d.text(
                        (x + 10, y - 9),
                        str(key[1]),
                        font=small,
                        fill="#b8deff" if key[0] == "blue" else "#fff09a",
                        stroke_width=1,
                        stroke_fill="#16251f",
                    )
            d.rectangle((0, 0, width, 51), fill="#10212b")
            d.text((15, 10), "5v5   Touchline restart", font=title, fill="white")
            d.text(
                (width - 15, 15),
                f"{f['time'] - trace[0]['time']:.1f} s     {cmd.replace('_', ' ')}",
                anchor="ra",
                font=body,
                fill="#b6daed",
            )
            d.text((15, height + 68), label, font=body, fill="#fff2ca")
            enc.stdin.write(canvas.tobytes())
        enc.stdin.close()
        if enc.wait():
            raise RuntimeError("Native frame encoding failed")
    subprocess.run(
        [
            "ffmpeg",
            "-v",
            "error",
            "-y",
            "-i",
            str(movie),
            "-vf",
            "setpts=0.5*PTS,fps=12.5,scale=704:-2,split[a][b];[a]palettegen[p];[b][p]paletteuse=dither=bayer",
            "-loop",
            "0",
            str(dest / "touchline-restart.gif"),
        ],
        check=True,
    )
    print(len(frames), "native frames")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    film(a.run, a.output)
