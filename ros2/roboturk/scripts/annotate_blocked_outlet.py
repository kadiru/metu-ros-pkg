"""Crop native Webots pixels to the scoring play and mark its trace-verified passing decision."""

from pathlib import Path
import tempfile

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from build_contest_preview import ffmpeg, make_gif


def pixel(point):
    return (640 + 93.5 * point[0], 360 - 93.5 * point[1])


def font(size):
    for f in ("DejaVuSans.ttf", "/System/Library/Fonts/Helvetica.ttc"):
        try:
            return ImageFont.truetype(f, size)
        except OSError:
            continue
    return ImageFont.load_default(size=size)


def play_preview(movie, dest, trace, row, start, duration):
    states = np.load(trace)["states"]
    chain = row["chain"]
    offset = 3 * (chain["team"] == "yellow")
    passer = offset + row["passer"]
    receiver = offset + row["receiver"]
    direction = (1 if row["seed"] % 2 == 0 else -1) * (1 if offset == 0 else -1)
    team = chain["team"].title()
    end = chain["goal"] + 1
    players = states[:end, 4:].reshape(-1, 6, 3)
    points = np.vstack(
        [
            states[:end, :2],
            players[:, passer, :2],
            players[:, receiver, :2],
            [[4.52 * direction, 0]],
        ]
    )
    coords = np.asarray([pixel(p) for p in points])
    center = (coords.max(0) + coords.min(0)) / 2
    width = int(max(510, np.ptp(coords[:, 0]) + 100, (np.ptp(coords[:, 1]) + 90) * 2.0))
    width = min(width, 1050)
    height = width // 2
    x = int(np.clip(center[0] - width / 2, 0, 1280 - width))
    y = int(np.clip(center[1] - height / 2, 0, 720 - height))
    green = (130, 234, 209)
    gold = (255, 202, 102)
    text_color = (239, 245, 248)
    title_font = font(24)
    caption_font = font(22)
    label_font = font(17)

    def screen(point):
        px, py = pixel(point)
        return ((px - x) * 960 / width, 52 + (py - y) * 480 / height)

    with tempfile.TemporaryDirectory() as tmp:
        temp = Path(tmp)
        ffmpeg(
            "-ss",
            start,
            "-t",
            duration,
            "-i",
            movie,
            "-vf",
            "fps=12.5",
            "-start_number",
            "0",
            temp / "frame-%03d.png",
        )
        frames = sorted(temp.glob("frame-*.png"))
        for i, path in enumerate(frames):
            t = i / 12.5
            sample = min(int(round(t * 100)), len(states) - 1)
            s = states[sample]
            ball = s[:2]
            rs = s[4:].reshape(6, 3)
            field = (
                Image.open(path)
                .convert("RGB")
                .crop((x, y, x + width, y + height))
                .resize((960, 480), Image.Resampling.LANCZOS)
            )
            img = Image.new("RGB", (960, 585), (17, 29, 39))
            img.paste(field, (0, 52))
            draw = ImageDraw.Draw(img)
            draw.text(
                (18, 10),
                f"RoboTurk  /  3v3 Webots     {team} {row['passer']} to {team} {row['receiver']}",
                font=title_font,
                fill=text_color,
            )
            draw.text((936, 13), f"{t:04.1f} s", font=caption_font, fill=text_color, anchor="ra")
            if sample < chain["recovery"]:
                caption = "Opening contest"
            elif sample < row["decision"] * 4:
                caption = f"{team} {row['passer']} wins sole possession"
            elif sample < row["kick"] * 4:
                caption = "Forward lane blocked     /     Open passing lane"
                rival = rs[:3] if offset else rs[3:]
                delta = np.array([4.53 * direction, 0]) - ball
                delta /= np.linalg.norm(delta)
                rel = rival[:, :2] - ball
                along = rel @ delta
                gaps = np.abs(rel[:, 0] * delta[1] - rel[:, 1] * delta[0])
                valid = np.flatnonzero((along > 0.15) & (along < 1.69) & (gaps <= 0.24))
                if len(valid):
                    bidx = valid[np.argmin(gaps[valid])]
                    point = screen(rival[bidx, :2])
                    draw.line([screen(ball), point], fill=gold, width=3)
                    draw.ellipse(
                        (point[0] - 10, point[1] - 10, point[0] + 10, point[1] + 10),
                        outline=gold,
                        width=2,
                    )
                a, b = screen(ball), screen(rs[receiver, :2])
                for k in range(0, 12, 2):
                    draw.line(
                        [
                            (a[0] + (b[0] - a[0]) * v / 12, a[1] + (b[1] - a[1]) * v / 12)
                            for v in (k, k + 1)
                        ],
                        fill=green,
                        width=3,
                    )
            elif sample <= (row["end"] + 1) * 4:
                caption = f"{team} {row['passer']} passes to {team} {row['receiver']}"
            elif sample < chain["shot"] * 4:
                caption = f"{team} {row['receiver']} controls the pass and turns toward goal"
            elif sample < chain["goal"]:
                caption = f"{team} {chain['scorer']} shoots"
            else:
                caption = f"GOAL   /   {team} {chain['scorer']} finishes the same possession"
            if row.get("verified_lead") and row["kick"] * 4 <= sample <= row["contact"] + 5:
                # Reveal only the target actually committed when the native kicker fired.
                target = screen(row["target"])
                origin = screen(states[row["kick"] * 4 - 1, :2])
                for k in range(0, 16, 2):
                    draw.line(
                        [
                            (
                                origin[0] + (target[0] - origin[0]) * v / 16,
                                origin[1] + (target[1] - origin[1]) * v / 16,
                            )
                            for v in (k, k + 1)
                        ],
                        fill=green,
                        width=2,
                    )
                draw.ellipse(
                    (target[0] - 8, target[1] - 8, target[0] + 8, target[1] + 8),
                    outline=green,
                    width=3,
                )
                draw.text(
                    (target[0] + 11, target[1] + 8),
                    "Lead target",
                    font=label_font,
                    fill=green,
                    stroke_width=2,
                    stroke_fill=(20, 36, 41),
                )
                caption = f"{team} {row['passer']} leads the run   /   {row['lead']:.2f} m ahead at release"
            if chain["recovery"] <= sample < chain["goal"]:
                for actor in [passer, receiver]:
                    px, py = screen(rs[actor, :2])
                    draw.text(
                        (px + 10, py - 18),
                        f"{team[0]}{actor % 3}",
                        font=label_font,
                        fill=text_color,
                        stroke_width=2,
                        stroke_fill=(20, 36, 41),
                    )
            draw.text(
                (18, 546),
                caption,
                font=caption_font,
                fill=gold if sample >= chain["goal"] else text_color,
            )
            img.save(path)
        encoded = temp / "annotated.mkv"
        ffmpeg(
            "-framerate",
            "12.5",
            "-start_number",
            "0",
            "-i",
            temp / "frame-%03d.png",
            "-c:v",
            "ffv1",
            encoded,
        )
        info = make_gif(encoded, dest, 0, len(frames) / 12.5, 12.5)
        return info, [x, y, width, height]
