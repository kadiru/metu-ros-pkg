"""Join two audited native excerpts at normal speed, with a short break between games."""

import argparse
import json
from pathlib import Path
import tempfile

from build_contest_preview import digest, ffmpeg, make_gif, action_inset
from build_three_preview import probe


def main():
    p = argparse.ArgumentParser()
    p.add_argument("media", type=Path)
    p.add_argument("demo", type=Path)
    a = p.parse_args()
    folder = a.media
    ordinary = json.loads((folder / "recovery-finish.json").read_text())
    fresh = json.loads((folder / "recovery-to-goal.json").read_text())["excerpt"]
    assert ordinary["play"]["dribble_m"] >= 0.18 and fresh["play"]["passes"]
    i = fresh["match"] - 1
    e = fresh["play"]
    seconds = json.loads((a.demo / "launch.json").read_text())["seconds"]
    start = fresh["source_start"] - i * seconds
    team_offset = 0 if e["team"] == "blue" else 3
    inset = action_inset(
        a.demo / f"trace-{i:03}.npz",
        start,
        e["shot"] * 0.04,
        e["winner"],
        team_offset + e["scorer"],
    )
    specs = [
        (
            "recovery-finish.mp4",
            ordinary["source_start"],
            ordinary["source_end"],
            ordinary["excerpt"]["native_zoom_crop_and_overlay_x"],
        ),
        ("recovery-to-goal.mp4", fresh["source_start"], fresh["source_end"], inset),
    ]
    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        inputs = []
        filters = []
        for j, (name, begin, end, (x, y, w, h, px)) in enumerate(specs):
            inputs.extend(["-ss", begin, "-t", end - begin, "-i", folder / name])
            filters.extend(
                [
                    f"[{j}:v]fps=25,split[a{j}][b{j}]",
                    f"[a{j}]scale=960:540[wide{j}]",
                    f"[b{j}]crop={w}:{h}:{x}:{y},scale=450:-1,pad=iw+4:ih+4:2:2:color=0xE5D992[z{j}]",
                    f"[wide{j}][z{j}]overlay={px}:100,setsar=1,setpts=PTS-STARTPTS[v{j}]",
                ]
            )
        filters += [
            "color=c=black:s=960x540:r=25:d=0.32[break]",
            "[v0][break][v1]concat=n=3:v=1:a=0[out]",
        ]
        movie = tmp / "excerpts.mp4"
        ffmpeg(
            *inputs,
            "-filter_complex_threads",
            "1",
            "-filter_complex",
            ";".join(filters),
            "-map",
            "[out]",
            "-an",
            "-c:v",
            "libx264",
            "-crf",
            "16",
            "-pix_fmt",
            "yuv420p",
            movie,
        )
        seconds = float(probe(movie)["format"]["duration"])
        gif = make_gif(movie, folder / "recovery-choices.gif", 0, seconds, 12.5)
    sources = [
        dict(file=k, sha256=digest(folder / k), start=s, end=e, inset=list(c))
        for k, s, e, c in specs
    ]
    (folder / "recovery-choices.json").write_text(
        json.dumps(dict(gif=gif, sources=sources, break_seconds=0.32, speed=1), indent=2) + "\n"
    )
    print(gif)


if __name__ == "__main__":
    main()
