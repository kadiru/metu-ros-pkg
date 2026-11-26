"""Join the three verified old/new replay pairs in the original match order."""

import argparse
from pathlib import Path
import tempfile
from build_contest_preview import digest, ffmpeg, make_gif
from train_takeaway import read, save
from build_three_preview import probe


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("media", type=Path)
    d = p.parse_args().media.resolve()
    clips = [
        ("attacking-repair", "attacking-repair"),
        ("takeaway-repair", "repair"),
        ("deep-selected", "deep-selected"),
    ]
    rows = []
    with tempfile.TemporaryDirectory() as tmp:
        listing = Path(tmp) / "clips.txt"
        paths = []
        for stem, receipt in clips:
            film = d / (stem + ".mp4")
            r = read(d / (receipt + ".json"))
            assert digest(film) == r["sha256"] and r["selected"]["outcome"] == "goal"
            assert r["training_set"] and r["exact_events"]
            rows.append(
                dict(
                    seed=r["seed"],
                    sample=r["selected"]["sample"],
                    goal=r["selected"]["goal"],
                    clip=film.name,
                )
            )
            paths.append("file '" + str(film).replace("'", "'\\''") + "'")
        listing.write_text("\n".join(paths) + "\n")
        combined = d / "three-repaired-takeaways.mp4"
        ffmpeg("-f", "concat", "-safe", 0, "-i", listing, "-c", "copy", combined)
    duration = float(probe(combined)["format"]["duration"])
    gif = make_gif(combined, combined.with_suffix(".gif"), 0, duration, 12.5)
    save(
        d / "three-repairs.json",
        dict(original_training_cases=rows, seconds=duration, sha256=digest(combined), gif=gif),
    )


if __name__ == "__main__":
    main()
