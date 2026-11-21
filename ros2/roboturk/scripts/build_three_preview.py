#!/usr/bin/env python3
"""Encode every frame interval of the native 3v3 movie in a README GIF."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def probe(path):
    return json.loads(
        subprocess.check_output(
            ["ffprobe", "-v", "error", "-show_streams", "-show_format", "-of", "json", str(path)]
        )
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("recording", type=Path)
    p.add_argument("--media", type=Path, default=Path("media"))
    a = p.parse_args()
    source = a.recording / "match.mp4"
    games = json.loads((a.recording / "results.json").read_text())
    audit = json.loads((a.recording / "audit.json").read_text())
    assert audit["passed"] and len(games) == 2
    native = probe(source)
    seconds = float(native["format"]["duration"])
    # Referee resets settle for 10 ms of native recording time.
    assert 119.9 <= seconds <= 121
    a.media.mkdir(parents=True, exist_ok=True)
    movie = a.media / "three-recipient-self-play.mp4"
    gif = movie.with_suffix(".gif")
    if movie.exists() or gif.exists():
        p.error("Output media already exist")
    shutil.copyfile(source, movie)

    def run(cmd):
        return subprocess.run(cmd, check=True)

    run(["ffmpeg", "-v", "error", "-i", str(movie), "-f", "null", "-"])
    run(
        [
            "ffmpeg",
            "-v",
            "error",
            "-threads",
            "1",
            "-i",
            str(movie),
            "-filter_complex_threads",
            "1",
            "-filter_complex",
            "fps=10,scale=960:-1:flags=lanczos,split[a][b];"
            "[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:diff_mode=rectangle",
            "-loop",
            "0",
            str(gif),
        ]
    )
    run(["ffmpeg", "-v", "error", "-ignore_loop", "1", "-i", str(gif), "-f", "null", "-"])
    info = probe(gif)
    frames = int(info["streams"][0]["nb_frames"])
    assert abs(frames / 10 - seconds) < 0.11
    assert b"NETSCAPE2.0\x03\x01\x00\x00" in gif.read_bytes()

    def sha(f):
        return hashlib.sha256(f.read_bytes()).hexdigest()

    assert sha(source) == sha(movie)
    receipt = dict(
        native_seconds=seconds,
        native_frames=int(native["streams"][0]["nb_frames"]),
        native_sha256=sha(movie),
        gif_seconds=frames / 10,
        gif_sha256=sha(gif),
        gif_frames=frames,
        gif_fps=10,
        cuts=0,
        speed=1,
        full_decodes=True,
    )
    movie.with_suffix(".json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(receipt)


if __name__ == "__main__":
    main()
