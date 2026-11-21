#!/usr/bin/env python3
"""Encode full native game footage and an uninterrupted, trace-linked scoring play."""

import argparse
import json
from pathlib import Path
import shutil
from build_contest_preview import digest, ffmpeg, make_gif
from build_three_preview import probe


def main():
    p = argparse.ArgumentParser()
    p.add_argument("demo", type=Path)
    p.add_argument("--media", type=Path, default=Path("media"))
    a = p.parse_args()
    cfg = json.loads((a.demo / "launch.json").read_text())
    assert json.loads((a.demo / "audit.json").read_text())["passed"]
    movie = a.media / "recovery-to-goal.mp4"
    a.media.mkdir(parents=True, exist_ok=True)
    source = a.demo / "match.mp4"
    shutil.copyfile(source, movie)
    assert digest(source) == digest(movie)
    ffmpeg("-i", movie, "-f", "null", "-")
    native = probe(movie)
    duration = float(native["format"]["duration"])
    assert abs(duration - cfg["cases"] * cfg["seconds"]) < 1
    moments = []
    for i in range(cfg["cases"]):
        game = json.loads((a.demo / f"game-{i:03}.json").read_text())
        for e in game["conversions"]:
            if "goal" in e:
                moments.append(
                    (
                        (
                            bool(e["passes"]),
                            e["dribble_m"] >= 0.18,
                            i == 2 or e["start"] > 0,
                            -i,
                            -e["start"],
                        ),
                        i,
                        e,
                    )
                )
    _, i, e = max(moments, key=lambda v: v[0])
    start = max(0, e["start"] * 0.01 - 0.7)
    end = min(cfg["seconds"], e["goal"] * 0.01 + 1.2)
    begin = i * cfg["seconds"] + start
    full = make_gif(movie, movie.with_suffix(".gif"), 0, duration, 10)
    clip = make_gif(movie, a.media / "recovery-goal-play.gif", begin, end - start, 12.5)
    clip.update(
        match=i + 1,
        seed=cfg["first"] + i,
        source_start=begin,
        source_end=i * cfg["seconds"] + end,
        play=e,
    )
    receipt = dict(
        native_sha256=digest(movie),
        native_frames=int(native["streams"][0]["nb_frames"]),
        native_seconds=duration,
        full=full,
        excerpt=clip,
        speed=1,
        full_cuts=0,
        fully_decoded=True,
    )
    movie.with_suffix(".json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
