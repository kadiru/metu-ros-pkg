"""Encode the first independently verified blocked-lane passing goal from fresh demo games."""

import argparse
import json
from pathlib import Path
import shutil
from build_contest_preview import digest, ffmpeg, make_gif
from annotate_blocked_outlet import play_preview
from build_three_preview import probe


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("media", type=Path)
    args = parser.parse_args()
    folder = args.recording
    cfg = json.loads((folder / "launch.json").read_text())
    audit = json.loads((folder / "audit.json").read_text())
    if not audit["passing_goals"]:
        raise ValueError("No verified opening pass goal in this recording")
    i, row = next((i, r) for i, r in enumerate(audit["rows"]) if r["qualified"])
    chain = row["chain"]
    dest = args.media
    dest.mkdir(parents=True, exist_ok=True)
    movie = dest / "blocked-outlet.mp4"
    shutil.copyfile(folder / "match.mp4", movie)
    ffmpeg("-i", movie, "-f", "null", "-")
    seconds = float(probe(movie)["format"]["duration"])
    assert abs(seconds - cfg["cases"] * cfg["seconds"]) < 0.2
    end = min(cfg["seconds"], chain["goal"] * 0.01 + 1.0)
    full = make_gif(movie, dest / "all-openings.gif", 0, seconds, 10)
    excerpt, crop = play_preview(
        movie,
        dest / "open-teammate.gif",
        folder / f"trace-{i:03}.npz",
        row,
        i * cfg["seconds"],
        end,
    )
    data = dict(
        file=movie.name,
        sha256=digest(movie),
        seconds=seconds,
        full=full,
        excerpt=excerpt,
        excerpt_start=i * cfg["seconds"],
        excerpt_game=i,
        opening=row,
        native_pixel_crop=crop,
        speed=1,
    )
    (dest / "recording.json").write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps(data, indent=2))


if __name__ == "__main__":
    main()
