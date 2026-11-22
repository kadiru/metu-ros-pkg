"""Caption native Webots frames using the verified official score and restart log."""

import argparse
import json
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from verify import load_log, verify_active


def font(size):
    for name in ("DejaVuSans.ttf", "/System/Library/Fonts/Supplemental/Arial.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default(size=size)


def film(folder, dest):
    dest.mkdir(exist_ok=True, parents=True)
    audit = verify_active(folder)
    g = audit["decisions"][0]
    log = load_log(folder / "match")
    frames = log["frames"]
    states = np.load(folder / "match/positions.npz")["states"]
    idx = np.flatnonzero(
        (abs(states[: g["score_tick"], 0]) > 4.5 + 0.0215)
        & (abs(states[: g["score_tick"], 1]) < 0.5 - 0.0215)
    )
    if not len(idx):
        raise ValueError("No physical goal-mouth crossing in native match")
    cross = int(idx[0])
    audit["first_goal_crossing_tick"] = cross
    (dest / "active.json").write_text(json.dumps(audit, indent=2) + "\n")
    w, h = 792, 500
    reader = [
        "ffmpeg",
        "-v",
        "error",
        "-i",
        str(folder / "match/native.mp4"),
        "-vf",
        f"crop=990:625:143:45,scale={w}:{h}",
        "-pix_fmt",
        "rgb24",
        "-f",
        "rawvideo",
        "-",
    ]
    movie = dest / "webots-autoref.mp4"
    writer = [
        "ffmpeg",
        "-v",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-s",
        f"{w}x{h + 90}",
        "-r",
        "25",
        "-pix_fmt",
        "rgb24",
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
    title, body = font(24), font(18)
    with (
        subprocess.Popen(reader, stdout=subprocess.PIPE) as src,
        subprocess.Popen(writer, stdin=subprocess.PIPE) as out,
    ):
        n = 0
        while data := src.stdout.read(w * h * 3):
            if n >= len(frames):
                raise ValueError("Unexpected extra native frames")
            f = frames[n]
            blue, yellow = f["score"]
            canvas = Image.new("RGB", (w, h + 90), "#101b24")
            canvas.paste(Image.frombytes("RGB", (w, h), data), (0, 54))
            d = ImageDraw.Draw(canvas)
            d.text((16, 10), f"Blue  {blue}   –   {yellow}  Yellow", font=title, fill="white")
            d.text(
                (w - 16, 14),
                f"{n * 0.04:.1f} s    {f['ssl_command'].replace('_', ' ') or 'WAITING'}",
                font=body,
                fill="#a9c8df",
                anchor="ra",
            )
            label = "Open play"
            if cross <= n < g["score_tick"]:
                label = "Ball crosses the goal line"
            elif g["score_tick"] <= n < g["prepare_tick"]:
                label = f"TIGERs reports the goal · Game Controller awards {g['scorer'].title()} one point"
            elif g["prepare_tick"] <= n < g["restart_tick"]:
                label = f"{g['kickoff'].title()} prepares for kickoff"
            elif g["restart_tick"] <= n < g["open_play_tick"]:
                label = f"{g['kickoff'].title()} takes the kickoff"
            elif n >= g["open_play_tick"]:
                label = "Ball back in play · both teams released"
            d.text((16, h + 60), label, font=body, fill="#f8e0a0")
            out.stdin.write(canvas.tobytes())
            n += 1
        out.stdin.close()
        if src.wait() or out.wait() or abs(n - len(frames)) > 2:
            raise RuntimeError(f"Native movie/telemetry mismatch ({n}/{len(frames)})")
    start = max(0, cross * 0.04 - 3)
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
            "-filter_complex",
            "fps=12.5,scale=620:-1,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer",
            "-loop",
            "0",
            str(dest / "goal-kickoff.gif"),
        ],
        check=True,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("output", type=Path)
    args = p.parse_args()
    film(args.run, args.output)
