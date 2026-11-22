"""Film and verify the two old matches containing all three clean takeaways."""

import argparse
from pathlib import Path
import tempfile
import numpy as np
from PIL import Image, ImageDraw
from annotate_blocked_outlet import font, pixel
from build_contest_preview import ffmpeg, make_gif, digest
from train_takeaway import OLD, ROOT, WORLD, WEBOTS, historical, read, save, sha, run_matches


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("media", type=Path)
    args = parser.parse_args()
    folder = args.folder.resolve()
    folder.mkdir(parents=True, exist_ok=True)
    args.media.mkdir(parents=True, exist_ok=True)
    cases = historical()
    captions = [
        "Blue 0 receives the pass. A defender gets the next touch.",
        "Blue 0 lines up a shot. The ball rolls wide before the kick.",
        "The opponent touches the ball again after 0.14 s.",
    ]
    sources = {}
    for f, g, r in cases:
        seed = g["seed"]
        if seed in sources:
            continue
        d = folder / str(seed)
        if not (d / "match.mp4").exists():
            d.mkdir(exist_ok=True)
            cfg = dict(
                root=str(ROOT),
                models=[read(OLD / "policy.json")],
                schedule=[[seed, 0, 0]],
                seconds=90,
                cases=1,
                press=True,
                record=True,
                world=str(WORLD.relative_to(ROOT)),
                webots_sha256=sha(WEBOTS),
            )
            save(d / "launch.json", cfg)
            run_matches(d, cfg, WEBOTS, WORLD, True)
        s = np.load(d / "trace-000.npz")["states"]
        movie = read(d / "game-000.json")
        assert np.array_equal(
            s,
            np.load(f.with_name(f.name.replace("game", "trace").replace("json", "npz")))["states"],
        )
        assert all(movie[k] == v for k, v in g.items() if k not in ("blue_model", "yellow_model"))
        sources[seed] = (d / "match.mp4", s)
    dest = args.media / "three-takeaway-losses.mp4"
    clips = []
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        count = 0
        for i, (_, g, r) in enumerate(cases):
            source, states = sources[g["seed"]]
            start = max(0, (r["sample"] + 1) / 100 - 0.65)
            end = min(90, (r["end"] + 1) / 100 + 0.75)
            ffmpeg(
                "-ss",
                start,
                "-i",
                source,
                "-t",
                end - start,
                "-vf",
                "fps=12.5",
                "-start_number",
                0,
                tmp / f"source-{i}-%04d.png",
            )
            images = sorted(tmp.glob(f"source-{i}-*.png"))
            clips.append(dict(seed=g["seed"], start=start, end=end, frames=len(images), row=r))
            for j, img in enumerate(images):
                sec = start + j / 12.5
                frame = Image.open(img).convert("RGB")
                canvas = Image.new("RGB", (960, 592), (17, 30, 39))
                draw = ImageDraw.Draw(canvas)
                canvas.paste(
                    frame.crop((176, 64, 1104, 656)).resize((812, 518), Image.Resampling.LANCZOS),
                    (74, 35),
                )
                k = min(round(sec * 100), r["end"])
                x, y = pixel(states[k, :2])
                x = int(np.clip(x - 65, 0, 1150))
                y = int(np.clip(y - 44, 0, 632))
                zoom = frame.crop((x, y, x + 130, y + 88)).resize(
                    (234, 158), Image.Resampling.LANCZOS
                )
                ix = 680 if x < 640 else 28
                draw.rectangle((ix - 2, 53, ix + 235, 214), fill=(241, 201, 111))
                canvas.paste(zoom, (ix, 55))
                draw.text(
                    (20, 5),
                    f"Original clean takeaway {i + 1} of 3     /     Blue {r['winner'] % 3}",
                    font=font(22),
                    fill="white",
                )
                draw.text(
                    (935, 5), f"{sec:.2f} s", font=font(21), fill=(241, 201, 111), anchor="ra"
                )
                draw.text((20, 557), captions[i], font=font(20), fill="white")
                canvas.save(tmp / f"clip-{count:04d}.png")
                count += 1
        ffmpeg(
            "-framerate",
            12.5,
            "-i",
            tmp / "clip-%04d.png",
            "-c:v",
            "libx264",
            "-crf",
            "19",
            "-pix_fmt",
            "yuv420p",
            dest,
        )
        gif = make_gif(dest, dest.with_suffix(".gif"), 0, count / 12.5, 12.5)
    save(
        args.media / "three-losses.json",
        dict(
            clips=clips,
            source_seeds=list(sources),
            exact_samples_per_source=9000,
            sha256=digest(dest),
            gif=gif,
        ),
    )


if __name__ == "__main__":
    main()
