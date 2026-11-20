#!/usr/bin/env python3
"""Build normal-speed README previews from existing native Webots recordings."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
CLIPS = (
    ("tdp-skills-demo", "tdp-skills-preview", 0.0, 30.8),
    ("learned-decisions-self-play-demo", "learned-pass-score-preview", 6.84, 11.76),
    ("continuous-self-play-demo", "continuous-goal-preview", 64.0, 16.0),
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    args = parser.parse_args()
    records = []
    for source_name, output_name, start, duration in CLIPS:
        source = ROOT / "media" / (source_name + ".mp4")
        output = ROOT / "media" / (output_name + ".gif")
        command = [
            args.ffmpeg,
            "-v",
            "error",
            "-y",
            "-threads",
            "1",
            "-ss",
            str(start),
            "-t",
            str(duration),
            "-i",
            str(source),
            "-filter_complex_threads",
            "1",
            "-filter_complex",
            "fps=12.5,scale=960:-1:flags=lanczos,split[a][b];"
            "[a]palettegen=stats_mode=diff[p];"
            "[b][p]paletteuse=dither=bayer:bayer_scale=3:diff_mode=rectangle",
            "-an",
            "-loop",
            "0",
            str(output),
        ]
        subprocess.run(command, check=True)
        subprocess.run(
            [args.ffmpeg, "-v", "error", "-ignore_loop", "1", "-i", str(output), "-f", "null", "-"],
            check=True,
        )
        records.append(
            {
                "source": str(source.relative_to(ROOT)),
                "source_sha256": digest(source),
                "gif": str(output.relative_to(ROOT)),
                "gif_sha256": digest(output),
                "bytes": output.stat().st_size,
                "source_start_seconds": start,
                "source_duration_seconds": duration,
                "speed": 1,
                "fps": 12.5,
                "width": 960,
                "height": 540,
                "loop": "forever",
                "full_decode_passed": True,
            }
        )
        print(output.name, output.stat().st_size, "bytes", flush=True)
    (ROOT / "media/gif-previews.json").write_text(json.dumps(records, indent=2) + "\n")


if __name__ == "__main__":
    main()
