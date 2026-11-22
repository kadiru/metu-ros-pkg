"""Overlay the actual ten role assignments on a verified native Webots movie."""

import argparse
from bisect import bisect_right
from functools import lru_cache
from pathlib import Path
import subprocess

from PIL import Image, ImageDraw
from annotate_blocked_outlet import font
from audit_five_roles import audit, load_match
from build_contest_preview import make_gif
from build_three_preview import probe

FONT = lru_cache(maxsize=12)(font)
BG = "#101c26"
PANEL = "#1c2b36"
WHITE = "#f6f7f4"
MUTED = "#aebdc6"
BLUE = "#85baff"
YELLOW = "#ffd86b"
WIDTH, HEIGHT = 1160, 660
FW, FH = 806, 534
X, Y = 12, 72


def jersey(k):
    return f"{'B' if k < 5 else 'Y'}{k % 5}"


def labels(draw, points):
    placed = []
    for k, (x, y) in enumerate(points):
        color = BLUE if k < 5 else YELLOW
        offsets = [(10, -25), (-34, -25), (12, 8), (-34, 8), (10, -43), (-34, 25)]

        def box(dx, dy):
            left = max(X + 1, min(X + FW - 26, round(x + dx)))
            top = max(Y + 1, min(Y + FH - 20, round(y + dy)))
            return (left, top, left + 25, top + 19)

        def overlap(b):
            return sum(
                max(0, min(b[2], p[2]) - max(b[0], p[0]))
                * max(0, min(b[3], p[3]) - max(b[1], p[1]))
                for p in placed
            )

        rect = min((box(*off) for off in offsets), key=overlap)
        placed.append(rect)
        draw.line((x, y + 1, (rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2), fill=color, width=1)
        draw.rounded_rectangle(rect, 4, fill=BG, outline=color)
        draw.text((rect[0] + 12, rect[1] + 9), jersey(k), font=FONT(13), fill=color, anchor="mm")


def event_captions(log, report):
    captions = {0: "Kickoff"}
    for v in report["control_changes"]:
        if v["event"] == "turnover":
            team = "Blue" if v["after"] < 5 else "Yellow"
            captions[v["tick"]] = (
                f"{jersey(v['after'])} wins possession. {team} switches to attack."
            )
    for e in log["events"]:
        if e["event"] == "pass":
            captions[e["tick"] + 1] = f"{jersey(e['actor'])} passes to {jersey(e['receiver'])}."
        elif e["event"] == "shot":
            captions[e["tick"] + 1] = f"{jersey(e['actor'])} shoots."
        else:
            captions[e["tick"] + 1] = (
                f"Goal for {e['team'].title()}." if e["event"] == "goal" else "Ball out. Restart."
            )
    for v in report["confirmed_passes"]:
        captions[v["receive"]] = (
            f"{jersey(v['receiver'])} receives from {jersey(v['passer'])} and takes over as carrier."
        )
    return captions


def movie_offset(native, states, clocks, fps):
    """Align the native movie clock to the physics trace using visible jersey centers."""
    import numpy as np

    source = subprocess.Popen(
        [
            "ffmpeg",
            "-v",
            "error",
            "-i",
            str(native),
            "-vf",
            r"select=not(mod(n\,25))",
            "-fps_mode",
            "vfr",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-",
        ],
        stdout=subprocess.PIPE,
    )
    hits = []
    j = 0
    while raw := source.stdout.read(1280 * 720 * 3):
        img = np.frombuffer(raw, np.uint8).reshape(720, 1280, 3)
        sec = j * 25 / fps
        j += 1
        index = max(0, bisect_right(clocks, sec + 0.04) - 1)
        for k, (x, y, _) in enumerate(states[index, 4:].reshape(10, 3)):
            cx, cy = round(639.5 + 94.25 * x), round(359.5 - 94.2 * y)
            red, green, blue = (
                img[cy - 16 : cy + 17, cx - 16 : cx + 17].astype(float).transpose(2, 0, 1)
            )
            mask = (
                (blue > 1.4 * red) & (blue > 1.4 * green)
                if k < 5
                else (red > 1.4 * blue) & (green > 1.3 * blue)
            )
            py, px = np.where(mask)
            if 130 < len(px) < 260 and np.ptp(px) < 22 and np.ptp(py) < 22:
                hits.append((sec, k, cx + px.mean() - 16, cy + py.mean() - 16))
    source.stdout.close()
    if source.wait() or not hits:
        raise ValueError("Cannot align recorded jerseys to physics")

    def residual(offset):
        errors = []
        for sec, k, x, y in hits:
            idx = max(0, bisect_right(clocks, sec + offset + 1e-8) - 1)
            rx, ry = states[idx, 4 + 3 * k : 6 + 3 * k]
            errors.append(np.hypot(x - 639.5 - 94.25 * rx, y - 359.5 + 94.2 * ry))
        return float(np.mean(errors))

    error, offset = min((residual(j / 100), j / 100) for j in range(13))
    if error > 4:
        raise ValueError("Jersey overlays do not align with native footage")
    return offset, error, len(hits)


def render(recording, reference, media):
    import json
    import numpy as np

    log, s = load_match(recording)
    other, t = load_match(reference)
    if log != other or not np.array_equal(s, t):
        raise ValueError("Movie and independent match differ")
    report = audit(recording)
    media.mkdir(parents=True, exist_ok=True)
    frames = log["frames"]
    clocks = [r["clock"] for r in frames]
    moments = event_captions(log, report)
    times = sorted(moments)
    movie = media / "five-roles.mp4"
    native = recording / "native.mp4"
    stream = probe(native)["streams"][0]
    assert stream["avg_frame_rate"] == "25/1"
    fps = 25
    # Webots may deliver its first rendered image one display cycle after movie start.
    assert abs(float(stream["duration"]) - clocks[-1]) < 0.18
    offset, residual, centers = movie_offset(native, s, clocks, fps)
    decoder = subprocess.Popen(
        [
            "ffmpeg",
            "-v",
            "error",
            "-i",
            str(native),
            "-vf",
            f"crop=922:610:179:52,scale={FW}:{FH},eq=brightness=0.025:saturation=1.1",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-",
        ],
        stdout=subprocess.PIPE,
    )
    encoder = subprocess.Popen(
        [
            "ffmpeg",
            "-v",
            "error",
            "-y",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "rgb24",
            "-s",
            f"{WIDTH}x{HEIGHT}",
            "-r",
            str(fps),
            "-i",
            "-",
            "-c:v",
            "libx264",
            "-crf",
            "19",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(movie),
        ],
        stdin=subprocess.PIPE,
    )
    changed = [-1000] * 10
    old = [""] * 10
    last = -1

    def pixel(x, y, robot=False):
        return X + (639.5 + (94.25 if robot else 93.7) * x - 179) * FW / 922, Y + (
            359.5 - (94.2 if robot else 93.7) * y - 52
        ) * FH / 610

    try:
        for j in range(int(stream["nb_frames"])):
            raw = decoder.stdout.read(FW * FH * 3)
            if len(raw) != FW * FH * 3:
                raise ValueError("Incomplete native frame")
            i = max(0, bisect_right(clocks, j / fps + offset + 1e-8) - 1)
            f = frames[i]
            for z in range(max(1, last + 1), i + 1):
                for k, n in enumerate(frames[z]["roles"]):
                    prev = frames[z - 1]["roles"][k]
                    if prev != n:
                        changed[k] = z
                        old[k] = prev
            last = i
            img = Image.new("RGB", (WIDTH, HEIGHT), BG)
            d = ImageDraw.Draw(img)
            img.paste(Image.frombytes("RGB", (FW, FH), raw), (X, Y))
            d.text((16, 12), "RoboTurk", font=FONT(30), fill=WHITE)
            d.text((178, 22), "5v5   /   Role assignments", font=FONT(19), fill=MUTED)
            score = f["score"]
            d.text(
                (818, 19), f"BLUE  {score[0]}   –   {score[1]}  YELLOW", font=FONT(20), fill=WHITE
            )
            d.text(
                (1145, 17),
                f"{int(i * log['dt']) // 60:02d}:{int(i * log['dt']) % 60:02d}",
                anchor="ra",
                font=FONT(23),
                fill=WHITE,
            )
            d.line((12, 58, 1148, 58), fill="#374851")
            poses = s[i, 4:].reshape(10, 3)
            points = [pixel(x, y, True) for x, y, _ in poses]
            bx, by = pixel(*s[i, :2])
            d.ellipse((bx - 5, by - 5, bx + 5, by + 5), outline="#fff5df", width=2)
            labels(d, points)
            for side, color in enumerate((BLUE, YELLOW)):
                y = Y + side * 269
                x = 833
                d.rounded_rectangle((x, y, 1148, y + 261), 8, fill=PANEL)
                d.text(
                    (x + 14, y + 9), "BLUE" if side == 0 else "YELLOW", font=FONT(22), fill=color
                )
                d.text((1134, y + 11), f["phases"][side], font=FONT(20), fill=WHITE, anchor="ra")
                d.line((x + 13, y + 42, 1135, y + 42), fill="#42515b")
                for number in range(5):
                    k = 5 * side + number
                    row = y + 48 + number * 41
                    recent = i - changed[k] < 22
                    if recent:
                        d.rounded_rectangle((x + 6, row, 1140, row + 36), 4, fill="#314653")
                    d.text((x + 14, row + 8), jersey(k), font=FONT(16), fill=color)
                    d.text((x + 56, row + 5), f["roles"][k], font=FONT(21), fill=WHITE)
                    if recent:
                        d.text(
                            (1134, row + 10),
                            "was " + old[k],
                            font=FONT(14),
                            fill=MUTED,
                            anchor="ra",
                        )
            q = times[bisect_right(times, i) - 1]
            note = (
                moments[q]
                if i - q <= 64
                else (
                    "Blue attacks right   /   Yellow attacks left"
                    if log["sign"] == 1
                    else "Yellow attacks right   /   Blue attacks left"
                )
            )
            d.text((16, 616), note, font=FONT(23), fill=WHITE)
            encoder.stdin.write(img.tobytes())
    finally:
        decoder.stdout.close()
        encoder.stdin.close()
        if decoder.wait() or encoder.wait():
            raise RuntimeError("Video conversion failed")
    start, end = report["excerpt"]
    duration = (end - start) * log["dt"]
    gif = make_gif(movie, media / "five-roles.gif", max(0, clocks[start] - offset), duration, 12.5)
    (media / "recording.json").write_text(
        json.dumps(
            dict(
                seed=log["seed"],
                native_frames=int(stream["nb_frames"]),
                independent_replay_exact=True,
                movie_offset_s=offset,
                jersey_center_mean_error_px=residual,
                jersey_centers_checked=centers,
                excerpt_match_seconds=[round(start * log["dt"], 2), round(end * log["dt"], 2)],
                gif=gif,
            ),
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("reference", type=Path)
    parser.add_argument("media", type=Path)
    args = parser.parse_args()
    render(args.recording, args.reference, args.media)
