#!/usr/bin/env python3
"""Normal-speed, full-length GIF from the uncut native strategy movie."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def probe(executable, path, frames=False):
    command = [executable, "-v", "error"]
    if frames:
        command += [
            "-ignore_loop",
            "1",
            "-show_frames",
            "-show_entries",
            "frame=pts_time,pkt_duration_time,duration_time",
        ]
    else:
        command += ["-show_streams", "-show_format"]
    return json.loads(subprocess.check_output(command + ["-of", "json", str(path)]))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ffmpeg", default="ffmpeg")
    p.add_argument("--ffprobe", default="ffprobe")
    args = p.parse_args()
    source = ROOT / "media/learned-skills-self-play-demo.mp4"
    output = ROOT / "media/learned-skills-self-play-preview.gif"
    if output.exists():
        p.error("Keep the existing preview; remove only after explicitly choosing to rebuild")
    original = probe(args.ffprobe, source)
    subprocess.run(
        [
            args.ffmpeg,
            "-v",
            "error",
            "-threads",
            "1",
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
        ],
        check=True,
    )
    subprocess.run(
        [args.ffmpeg, "-v", "error", "-ignore_loop", "1", "-i", str(output), "-f", "null", "-"],
        check=True,
    )
    info = probe(args.ffprobe, output, frames=True)
    frames = info["frames"]
    assert frames and all(
        abs(float(f.get("pkt_duration_time", f.get("duration_time"))) - 0.08) < 1e-6 for f in frames
    )
    assert b"NETSCAPE2.0\x03\x01\x00\x00" in output.read_bytes()
    duration = len(frames) * 0.08
    assert abs(duration - float(original["format"]["duration"])) <= 0.081
    record = {
        "source": str(source.relative_to(ROOT)),
        "source_sha256": digest(source),
        "gif": str(output.relative_to(ROOT)),
        "gif_sha256": digest(output),
        "bytes": output.stat().st_size,
        "source_start_seconds": 0,
        "source_duration_seconds": float(original["format"]["duration"]),
        "gif_duration_seconds": duration,
        "frames": len(frames),
        "fps": 12.5,
        "width": 960,
        "height": 540,
        "speed": 1,
        "loop": "forever",
        "cuts": 0,
        "outcome_selection": False,
        "full_decode_passed": True,
    }
    (ROOT / "media/learned-skills-self-play-preview.json").write_text(
        json.dumps(record, indent=2) + "\n"
    )
    print(json.dumps(record))


if __name__ == "__main__":
    main()
