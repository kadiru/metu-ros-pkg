#!/usr/bin/env python3
"""Make normal-speed Webots GIFs and tie the excerpt to an audited pass."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

import numpy as np

from build_three_preview import probe


def ffmpeg(*args):
    subprocess.run(
        ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", *map(str, args)], check=True
    )


def make_gif(movie, dest, start, length, fps, inset=None):
    filters = f"[0:v]fps={fps},scale=960:-1:flags=lanczos"
    if inset:
        x, y, w, h, overlay_x = inset
        filters = (
            f"[0:v]fps={fps},split[base][detail];[base]scale=960:-1:flags=lanczos[wide];"
            f"[detail]crop={w}:{h}:{x}:{y},scale=450:-1:flags=lanczos,pad=iw+4:ih+4:2:2:color=0xE5D992[zoom];"
            f"[wide][zoom]overlay={overlay_x}:100"
        )
    with tempfile.TemporaryDirectory() as tmp:
        palette = Path(tmp) / "palette.png"
        inputs = ["-ss", str(start), "-t", str(length), "-i", movie]
        ffmpeg(
            *inputs,
            "-filter_complex_threads",
            "1",
            "-filter_complex",
            filters + ",palettegen=stats_mode=diff",
            "-frames:v",
            "1",
            palette,
        )
        ffmpeg(
            *inputs,
            "-i",
            palette,
            "-filter_complex_threads",
            "1",
            "-filter_complex",
            f"{filters}[a];[a][1:v]paletteuse=dither=bayer:diff_mode=rectangle",
            "-loop",
            "0",
            dest,
        )
    ffmpeg("-ignore_loop", "1", "-i", dest, "-f", "null", "-")
    stream = probe(dest)["streams"][0]
    assert abs(float(stream["duration"]) - length) < 0.13
    assert b"NETSCAPE2.0\x03\x01\x00\x00" in dest.read_bytes()
    return dict(
        file=dest.name,
        frames=int(stream["nb_frames"]),
        seconds=float(stream["duration"]),
        fps=fps,
        sha256=digest(dest),
    )


def action_inset(trace, start, end, actor, receiver):
    frames = np.load(trace)["states"][int(start * 100) : int(end * 100)]
    poses = frames[:, 4:].reshape(-1, 6, 3)
    xy = np.concatenate((frames[:, :2], poses[:, actor, :2], poses[:, receiver, :2]))
    pixels = (xy * np.array([0.073, -0.130]) + np.array([0.5, 0.5])) * np.array([1280, 720])
    center = (pixels.min(0) + pixels.max(0)) / 2
    width = int(max(270, np.ptp(pixels, axis=0)[0] + 76, (np.ptp(pixels, axis=0)[1] + 60) * 1.65))
    width = min(740, width // 2 * 2)
    height = int(width / 1.65) // 2 * 2
    x, y = np.clip(center - [width / 2, height / 2], [0, 0], [1280 - width, 720 - height]).astype(
        int
    )
    return [int(x) // 2 * 2, int(y) // 2 * 2, width, height, 18 if center[0] > 640 else 488]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("recording", type=Path)
    p.add_argument("--media", type=Path, default=Path("media"))
    a = p.parse_args()
    source = a.recording / "match.mp4"
    assert json.loads((a.recording / "audit.json").read_text())["passed"]
    native = probe(source)
    seconds = float(native["format"]["duration"])
    games = sorted(a.recording.glob("game-*.json"))
    assert len(games) == 2 and 179.9 <= seconds <= 181
    choices = []
    for i, path in enumerate(games):
        g = json.loads(path.read_text())
        for e in g["contest_metrics"]["contests"]:
            if "received" not in e:
                continue
            team = "blue" if e["winner"] < 3 else "yellow"
            support = [
                v
                for v in g["learned_actions"]
                if v["head"] == "outlet"
                and v["team"] == team
                and v["actor"] == e["winner"] % 3
                and e["start"] <= 4 * v["tick"] <= e["recovery"]
            ]
            designated = bool(support and support[-1]["mate"] == e["receiver"] % 3)
            natural = i > 0 or e["start"] > 0
            choices.append(((designated, natural, i, -e["start"]), i, e, g, team))
    rank, i, e, game, team = max(choices, key=lambda x: x[0])
    # Include play leading into the tie and the intended player's controlled reception.
    start = max(0, e["start"] / 100 - 1.5)
    end = min(game["seconds"], e["received"] * 0.04 + 2)
    offset = i * game["seconds"]
    a.media.mkdir(exist_ok=True, parents=True)
    movie = a.media / "contest-outlet-self-play.mp4"
    shutil.copyfile(source, movie)
    assert digest(movie) == digest(source)
    ffmpeg("-i", movie, "-f", "null", "-")
    full = make_gif(movie, movie.with_suffix(".gif"), 0, seconds, 10)
    inset = action_inset(a.recording / f"trace-{i:03}.npz", start, end, e["winner"], e["receiver"])
    excerpt = make_gif(
        movie, a.media / "contest-outlet-pass.gif", start + offset, end - start, 12.5, inset
    )
    excerpt["native_zoom_crop_and_overlay_x"] = inset
    excerpt.update(
        match=i + 1,
        seed=game["seed"],
        team=team,
        passer=e["winner"] % 3,
        receiver=e["receiver"] % 3,
        designated=rank[0],
        contest=e["start"] / 100,
        sole=e["recovery"] / 100,
        kick=e["pass_tick"] * 0.04,
        reception=e["received"] * 0.04,
        source_start=start + offset,
        source_end=end + offset,
    )
    receipt = dict(
        native_seconds=seconds,
        native_frames=int(native["streams"][0]["nb_frames"]),
        native_sha256=digest(movie),
        speed=1,
        cuts_in_full_movie=0,
        full_decodes=True,
        full=full,
        excerpt=excerpt,
    )
    movie.with_suffix(".json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
