"""Show the current Presser and the trained tackle in synchronized native Webots recordings."""

import argparse
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageEnhance

from fit_tackle import rows, starts
from render_waypoint_replay import font

WIDTH = 510
STATUS = {
    (True, False): "Carrier has the ball",
    (True, True): "Both rollers reach",
    (False, False): "Ball is loose",
    (False, True): "Presser has the ball",
}


def panel(folder, label, row):
    initial = row["initial"]
    keys = [r["key"] for r in initial["robots"]]
    presser = keys.index(row["presser"])
    files = sorted((folder / "frames").glob("*.jpg"))
    samples = row["trace"][::2]
    images = []
    trail = []
    if not files:
        raise ValueError(f"No native Webots frames in {folder}")
    for tick, (file, state, control) in enumerate(
        zip(files, samples, row["control"][::2], strict=True)
    ):
        with Image.open(file) as source:
            height = round(WIDTH * 9 / 16)
            resized = source.convert("RGB").resize(
                (WIDTH, round(WIDTH * source.height / source.width)), Image.Resampling.LANCZOS
            )
        top = (resized.height - height) // 2
        scene = ImageEnhance.Brightness(resized.crop((0, top, WIDTH, height + top))).enhance(1.35)
        zoom = WIDTH / (2 * (3.7 - 0.15) * math.tan(math.pi / 8))
        x = WIDTH / 2 + (state[4 + 3 * presser] - initial["ball"]["x"]) * zoom
        y = height / 2 - (state[5 + 3 * presser] - initial["ball"]["y"]) * zoom
        trail.append((x, y))
        draw = ImageDraw.Draw(scene)
        if len(trail) > 1:
            draw.line(trail, width=2, fill="#d3e4ed")
        draw.ellipse((x - 19, y - 19, x + 19, y + 19), outline="white", width=2)
        canvas = Image.new("RGB", (WIDTH, height + 78), "#10202b")
        canvas.paste(scene, (0, 39))
        draw = ImageDraw.Draw(canvas)
        draw.text((11, 7), label, font=font(20, True), fill="white")
        now = tick * 0.08
        draw.text((442, 10), f"{now:.2f} s", font=font(15), fill="white")
        status = STATUS[tuple(control)]
        if row["capture_s"] is not None and now >= row["capture_s"]:
            status = (
                "Clean takeaway confirmed" if row["clean_takeaway"] else "Possession after contact"
            )
        elif row["first_contact_s"] is not None and now >= row["first_contact_s"]:
            status += " · Body contact recorded"
        elif tuple(control) == (True, True) and any(
            0 <= now - t < 0.68 for t in row.get("sweep_times", [])
        ):
            status = "Presser pivots the roller sideways"
        draw.text((11, canvas.height - 28), status, font=font(15), fill="white")
        images.append(canvas)
    return images


def render(current, trained, destination, labels):
    baseline, candidate = rows(current / "trials.jsonl.gz"), rows(trained / "trials.jsonl.gz")
    if len(baseline) != 1 or starts(baseline) != starts(candidate):
        raise ValueError("Record the same single opening for both arms")
    left, right = (
        panel(current, labels[0], baseline[0]),
        panel(trained, labels[1], candidate[0]),
    )
    frames = []
    for first, second in zip(left, right, strict=True):
        frame = Image.new("RGB", (WIDTH * 2 + 3, first.height), "#acb6be")
        frame.paste(first)
        frame.paste(second, (WIDTH + 3, 0))
        frames.append(frame)
    captured = candidate[0]["capture_s"]
    if captured is not None:
        # Include the exchange and half a second of protected carry while both
        # players are still visible in the fixed close-up.
        frames = frames[: math.floor((captured + 0.50) / 0.08) + 1]
    # Webots applies the close camera after the first exported image.
    frames = frames[1:]
    sample = frames[::5]
    strip = Image.new("RGB", (frames[0].width, frames[0].height * len(sample)))
    for i, frame in enumerate(sample):
        strip.paste(frame, (0, i * frame.height))
    palette = strip.quantize(colors=224)
    images = [r.quantize(palette=palette, dither=Image.Dither.NONE) for r in frames]
    images[0].save(
        destination,
        save_all=True,
        append_images=images[1:],
        duration=[160] * (len(images) - 1) + [600],
        loop=0,
        optimize=True,
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("current", type=Path)
    p.add_argument("trained", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--labels", nargs=2, default=("Current Presser", "Trained ball-side tackle"))
    a = p.parse_args()
    render(a.current, a.trained, a.output, a.labels)
