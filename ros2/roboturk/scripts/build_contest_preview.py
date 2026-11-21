#!/usr/bin/env python3
"""Make normal-speed Webots GIFs and tie the excerpt to an audited pass."""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

from build_three_preview import probe


def ffmpeg(*args):
    subprocess.run(
        ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", *map(str, args)], check=True
    )


def make_gif(movie, dest, start, length, fps):
    filters = f"fps={fps},scale=960:-1:flags=lanczos"
    with tempfile.TemporaryDirectory() as tmp:
        palette = Path(tmp) / "palette.png"
        inputs = ["-ss", str(start), "-t", str(length), "-i", movie]
        ffmpeg(*inputs, "-vf", filters + ",palettegen=stats_mode=diff", "-frames:v", "1", palette)
        ffmpeg(
            *inputs,
            "-i",
            palette,
            "-filter_complex_threads",
            "1",
            "-filter_complex",
            f"[0:v]{filters}[a];[a][1:v]paletteuse=dither=bayer:diff_mode=rectangle",
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
            choices.append(((designated, i, -e["start"]), i, e, g, team))
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
    excerpt = make_gif(
        movie, a.media / "contest-outlet-pass.gif", start + offset, end - start, 12.5
    )
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
