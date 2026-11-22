"""Film the first verified goal from the best historical repair profile."""

import argparse
from pathlib import Path
import tempfile
import numpy as np
from PIL import Image, ImageDraw
from annotate_blocked_outlet import font
from build_contest_preview import digest, ffmpeg, make_gif
from train_takeaway import WEBOTS, WORLD, OLD, historical, read, save, run_matches


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder", type=Path)
    p.add_argument("media", type=Path)
    p.add_argument("old_film", type=Path)
    p.add_argument("searches", nargs="+", type=Path)
    p.add_argument("--sample", type=int, help="Film this historical recovery")
    p.add_argument("--stem", default="takeaway-repair")
    args = p.parse_args()
    choices = []
    for search in args.searches:
        rows = read(search / "options.json")
        if args.sample is not None:
            rows = [(i, r) for i, r in enumerate(rows) if r["sample"] == args.sample]
        else:
            rows = list(enumerate(rows))
        for j in sorted({r["profile"] for _, r in rows}):
            goals = [(i, r) for i, r in rows if r["profile"] == j and "goal" in r]
            choices.append((len(goals), search, j, goals))
    count, search, profile, goals = max(choices, key=lambda v: v[0])
    if not goals:
        p.error("No goal on the original takeaways")
    slot, row = goals[0]
    cfg = read(search / "launch.json")
    folder = args.folder.resolve()
    folder.mkdir(exist_ok=True, parents=True)
    args.media.mkdir(exist_ok=True, parents=True)
    seed = row["seed"]
    f, _, before = next(
        h for h in historical() if h[1]["seed"] == seed and h[2]["sample"] == row["sample"]
    )
    prior = args.old_film / str(seed)
    assert np.array_equal(
        np.load(prior / "trace-000.npz")["states"],
        np.load(OLD / "evaluation" / f.name.replace("game", "trace").replace("json", "npz"))[
            "states"
        ],
    )
    for k in ("schedule", "durations", "starts"):
        cfg[k] = [cfg[k][slot]]
    cfg.update(cases=1, record=True)
    save(folder / "launch.json", cfg)
    if not (folder / "match.mp4").exists():
        run_matches(folder, cfg, WEBOTS, WORLD, True)
    states = np.load(folder / "trace-000.npz")["states"]
    assert np.array_equal(states, np.load(search / f"trace-{slot:03}.npz")["states"])
    assert read(folder / "game-000.json") == read(search / f"game-{slot:03}.json")
    assert np.array_equal(
        states[: row["matched_samples"]],
        np.load(prior / "trace-000.npz")["states"][: row["matched_samples"]],
    )
    start = max(0, (row["sample"] + 1) / 100 - 0.8)
    end = max(before["end"], row["goal"]) / 100 + 0.65
    mp4 = args.media / (args.stem + ".mp4")
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for label, src in [("before", prior), ("after", folder)]:
            ffmpeg(
                "-ss",
                start,
                "-i",
                src / "match.mp4",
                "-t",
                end - start,
                "-vf",
                "fps=12.5",
                "-start_number",
                0,
                tmp / f"{label}-%04d.png",
            )
        n = min(len(list(tmp.glob(f"{s}-*.png"))) for s in ("before", "after"))
        for j in range(n):
            t = start + j / 12.5
            img = Image.new("RGB", (1120, 424), (17, 30, 39))
            draw = ImageDraw.Draw(img)
            for k, (label, caption, outcome, stamp) in enumerate(
                [
                    ("before", "55-goal team", before["outcome"].capitalize(), before["end"]),
                    ("after", "Trained on this takeaway", "Goal", row["goal"]),
                ]
            ):
                # Hold the last frame so both outcomes remain readable at the same clock.
                stop = min(j, max(0, int(((stamp + 1) / 100 - 0.04 - start) * 12.5)))
                frame = Image.open(tmp / f"{label}-{stop:04d}.png").convert("RGB")
                img.paste(
                    frame.crop((176, 64, 1104, 656)).resize((550, 352), Image.Resampling.LANCZOS),
                    (k * 560 + 5, 38),
                )
                draw.text((k * 560 + 12, 5), caption, font=font(22), fill="white")
                msg = (
                    f"{outcome} at {(stamp + 1) / 100:.2f} s"
                    if t >= (stamp + 1) / 100
                    else (
                        f"Blue {row['winner'] % 3} wins the ball"
                        if t >= (row["sample"] + 1) / 100
                        else "Ball in play"
                    )
                )
                draw.text((k * 560 + 12, 393), msg, font=font(20), fill=(240, 203, 122))
            draw.text((1105, 7), f"{t:.1f} s", anchor="ra", font=font(20), fill="white")
            img.save(tmp / f"edit-{j:04d}.png")
        ffmpeg(
            "-framerate",
            12.5,
            "-i",
            tmp / "edit-%04d.png",
            "-c:v",
            "libx264",
            "-crf",
            19,
            "-pix_fmt",
            "yuv420p",
            mp4,
        )
        gif = make_gif(mp4, mp4.with_suffix(".gif"), 0, n / 12.5, 12.5)
    save(
        args.media / ("repair.json" if args.stem == "takeaway-repair" else args.stem + ".json"),
        dict(
            seed=seed,
            training_set=True,
            source=search.name,
            profile=profile,
            goals=count,
            previous=before,
            selected=row,
            matched_recording_samples=len(states),
            exact_events=True,
            matched_before_intervention=row["matched_samples"],
            sha256=digest(mp4),
            gif=gif,
        ),
    )


if __name__ == "__main__":
    main()
