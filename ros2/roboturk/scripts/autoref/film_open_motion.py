"""Render native incident pairs and the first registered fresh game with verified events."""

import argparse
from io import BytesIO
import json
from pathlib import Path
import subprocess
from zipfile import ZipFile
import numpy as np
from PIL import Image, ImageDraw
from film import font
from train_open_motion import PROTOCOL, ROOT, read_game, save

BG = "#10212b"
WHITE = "#faf7f0"
BLUE = "#98d2ff"
YELLOW = "#ffe184"


def recording(folder, result=None):
    log, states = read_game(folder)
    if result is None:
        result = json.loads((folder / "official/result.json").read_text())
    images = sorted((folder / "frames").glob("*.jpg"))
    return native_camera(log, states, result, images)


def native_camera(log, states, result, images):
    assert len(images) == (len(states) + 1) // 2
    with Image.open(images[0]) as im:
        w, h = im.size
        r, g, b = np.asarray(im)[h // 3].astype(float).T
        green = np.flatnonzero((g > 1.5 * r) & (g > 1.35 * b))
    return log, states, result, images, ((w - 1) / 2, (h - 1) / 2, (green[-1] - green[0] + 1) / 10)


def pitch(rec, tick, center, field_width, shape, ids, trails=False):
    log, states, _, files, (cx, cy, unit) = rec
    width, height = shape
    left = cx + unit * (center[0] - field_width / 2)
    top = cy - unit * (center[1] + field_width * height / width / 2)
    scale = width / (field_width * unit)
    box = (round(left), round(top), round(left + width / scale), round(top + height / scale))
    with Image.open(files[tick // 2]) as im:
        panel = im.convert("RGB").crop(box).resize(shape, Image.Resampling.LANCZOS)
    d = ImageDraw.Draw(panel)

    def point(v, z):
        depth = 16.5 / (16.5 - z)
        return ((cx + depth * unit * v[0] - left) * scale, (cy - depth * unit * v[1] - top) * scale)

    for k in ids:
        color = BLUE if k < 5 else YELLOW
        trace = states[max(0, tick - 11) : tick + 1, 4 + 3 * k : 6 + 3 * k]
        pts = [point(v, 0.15) for v in trace]
        if trails and len(pts) > 1:
            d.line(pts, fill=color, width=2)
        x, y = pts[-1]
        d.text(
            (x + 8, y - 17),
            str(log["keys"][k][1]),
            font=font(16),
            fill=color,
            stroke_width=1,
            stroke_fill=BG,
        )
    x, y = point(states[tick, :2], 0.022)
    d.ellipse((x - 4, y - 4, x + 4, y + 4), outline="#ffa64b", width=2)
    return panel


def encode(path, frames, fps):
    iterator = iter(frames)
    first = next(iterator)
    w, h = first.size
    cmd = [
        "ffmpeg",
        "-v",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{w}x{h}",
        "-r",
        str(fps),
        "-i",
        "-",
        "-c:v",
        "libx264",
        "-preset",
        "fast",
        "-crf",
        "18",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(path),
    ]
    with subprocess.Popen(cmd, stdin=subprocess.PIPE) as stream:
        stream.stdin.write(first.tobytes())
        for im in iterator:
            stream.stdin.write(im.tobytes())
        stream.stdin.close()
        if stream.wait():
            raise RuntimeError("Film encoding failed")


def incident_film(dest, run):
    with ZipFile(ROOT / "policies/placement_v2/evidence.zip") as archive:
        with np.load(BytesIO(archive.read("natural-film/match/positions.npz"))) as f:
            original = f["states"]
    old_calls = json.loads((ROOT / "media/placement-traffic/open-match.json").read_text())[
        "controller_events"
    ]
    all_frames = []
    reports = []
    for n, case in enumerate(PROTOCOL["incidents"]):
        panels = [recording(run / str(n) / side) for side in ("baseline", "candidate")]
        switch = case["switch_tick"]
        ia, ib = case["blue"], 5 + case["yellow"]
        assert np.array_equal(original[: len(panels[0][1])], panels[0][1])
        assert np.array_equal(original[: switch + 1], panels[1][1][: switch + 1])
        windows = [[c for c in panel[2]["crashes"] if c["tick"] >= switch] for panel in panels]
        assert len(windows[0]) == 1 and not windows[1]
        call = windows[0][0]
        kind = "botCrashUnique" if n == 0 else "botCrashDrawn"
        original_call = old_calls[n][kind]
        data = call["event"][kind]
        assert abs(data["crashSpeed"] - original_call["crashSpeed"]) < 0.001
        if n == 0:
            assert (data["byTeam"], data["violator"], data["victim"]) == ("BLUE", ia, ib - 5)
        else:
            assert (data["botBlue"], data["botYellow"]) == (ia, ib - 5)
        center = list(original_call["location"][key] for key in ("x", "y"))
        gaps = []
        for _, states, *_ in panels:
            xy = states[:, 4:].reshape(-1, 10, 3)[:, :, :2]
            gaps.append(np.linalg.norm(xy[:, ia] - xy[:, ib], axis=1))
        report = dict(
            blue=ia,
            yellow=ib - 5,
            switch_s=switch * 0.04,
            matched_prefix_frames=switch + 1,
            original_min_m=float(min(gaps[0][switch:])),
            trained_min_m=float(min(gaps[1][switch:])),
            reproduced_crash_speed_mps=data["crashSpeed"],
            post_switch_calls=[len(q) for q in windows],
        )
        reports.append(report)
        frames = []
        for tick in range(max(0, switch - 8), len(gaps[0]), 2):
            canvas = Image.new("RGB", (900, 382), BG)
            d = ImageDraw.Draw(canvas)
            d.text((12, 8), f"Blue {ia} and Yellow {ib - 5}", font=font(21), fill=WHITE)
            d.text((887, 12), f"{tick * 0.04:.2f} s", font=font(17), fill=WHITE, anchor="ra")
            for j, panel in enumerate(panels):
                off = j * 450
                canvas.paste(
                    pitch(panel, tick, center, 2.75, (450, 270), [ia, ib], True), (off, 45)
                )
                d.text(
                    (off + 12, 320),
                    ["Current team", "Trained steering"][j],
                    font=font(18),
                    fill=WHITE,
                )
                label = f"Center distance   {gaps[j][tick] * 100:.0f} cm"
                if j == 0 and tick >= call["tick"]:
                    label = f"TIGERs crash call   {data['crashSpeed']:.2f} m/s"
                d.text((off + 12, 350), label, font=font(16), fill=YELLOW)
            d.line((450, 45, 450, 315), fill=BG, width=2)
            frames.append(canvas)
        frames[0].save(
            dest / f"encounter-{n + 1}.gif",
            save_all=True,
            append_images=frames[1:],
            duration=160,
            loop=0,
            optimize=True,
        )
        all_frames.extend(frames)
        all_frames.extend([frames[-1]] * 8)
    encode(dest / "two-encounters.mp4", all_frames, "25/2")
    save(dest / "incidents.json", reports)


def whole_match(dest, run):
    first = PROTOCOL["fresh_seeds_inclusive"][0]
    rec = recording(run / str(first) / "candidate")
    log, states, stats, _, _ = rec

    def frames():
        for tick in range(0, len(states), 2):
            frame = Image.new("RGB", (822, 668), BG)
            d = ImageDraw.Draw(frame)
            frame.paste(pitch(rec, tick, (0, 0), 10, (822, 576), range(10)), (0, 46))
            score = log["frames"][tick]["score"]
            d.text((12, 10), "5v5   Observed-motion steering", font=font(20), fill=WHITE)
            d.text(
                (811, 11),
                f"Blue {score[0]}   –   {score[1]} Yellow      {tick * 0.04:.1f} s",
                font=font(18),
                fill=WHITE,
                anchor="ra",
            )
            passes = [e for e in stats["passes"] if e["receive"] <= tick]
            crashes = [e for e in stats["crashes"] if e["tick"] <= tick]
            caption = f"{len(passes)} completed passes     {len(crashes)} TIGERs crash calls"
            if passes and tick - passes[-1]["receive"] < 23:
                a, b = passes[-1]["passer"], passes[-1]["receiver"]
                caption += f"      {a[0].title()} {a[1]} to {b[1]}"
            d.text((12, 635), caption, font=font(17), fill=YELLOW)
            yield frame

    movie = dest / "fresh-match.mp4"
    encode(movie, frames(), "25/2")
    if stats["goals"]:
        goal_time = stats["goals"][0]["tick"] * log["dt"]
        start = max(0.0, goal_time - 6.4)
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
                str(goal_time - start + 0.8),
                "-vf",
                "fps=12.5,scale=646:-2,split[a][b];[a]palettegen[p];[b][p]paletteuse=dither=bayer",
                "-loop",
                "0",
                str(dest / "first-goal.gif"),
            ],
            check=True,
        )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=("incidents", "fresh"))
    p.add_argument("run", type=Path)
    p.add_argument("dest", type=Path)
    args = p.parse_args()
    args.dest.mkdir(parents=True, exist_ok=True)
    (incident_film if args.mode == "incidents" else whole_match)(args.dest, args.run)
