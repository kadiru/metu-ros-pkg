#!/usr/bin/env python3
"""Compare a rendered scoring match to the held-out trace and encode its uninterrupted play."""

import argparse
from pathlib import Path
import shutil
import numpy as np
from audit_recovery_attack import panel
from build_contest_preview import ffmpeg, make_gif, digest, action_inset
from run_recovery_attack import read, save


def main():
    p = argparse.ArgumentParser()
    p.add_argument("showcase", type=Path)
    p.add_argument("qualification", type=Path)
    p.add_argument("--media", type=Path, default=Path("media"))
    a = p.parse_args()
    source_audit = read(a.qualification / "audit.json")
    assert source_audit["panels"]["ordinary"]["totals"]["assisted"] > 0
    cfg = read(a.showcase / "launch.json")
    assert cfg["mode"] == "showcase" and cfg["cases"] == 1
    result = panel(a.showcase)
    game = read(a.showcase / "game-000.json")
    candidates = [e for e in game["conversions"] if "goal" in e and e["passes"]]
    if not candidates:
        raise ValueError("No pass-assisted goal in the rendered match")
    e = max(candidates, key=lambda x: (x["dribble_m"] >= 0.18, len(x["passes"]), -x["goal"]))
    original = a.qualification / cfg["selected_from"]
    idx = game["seed"] - read(original / "launch.json")["first"]
    baseline = read(original / f"game-{idx:03}.json")
    with np.load(original / f"trace-{idx:03}.npz") as t, np.load(a.showcase / "trace-000.npz") as m:
        delta = float(np.max(abs(t["states"] - m["states"])))
    result.update(
        passed=True,
        source_panel_passed=source_audit["passed"],
        seed=game["seed"],
        max_trace_delta=delta,
        identical_events=game["events"] == baseline["events"],
        identical_conversions=game["conversions"] == baseline["conversions"],
    )
    save(a.showcase / "audit.json", result)
    movie = a.media / "recovery-pass-finish.mp4"
    a.media.mkdir(exist_ok=True, parents=True)
    shutil.copyfile(a.showcase / "match.mp4", movie)
    ffmpeg("-i", movie, "-f", "null", "-")
    assert digest(movie) == digest(a.showcase / "match.mp4")
    start = max(0, 0.01 * e["start"] - 0.65)
    end = min(cfg["seconds"], e["goal"] * 0.01 + 1.2)
    receiver = (0 if e["team"] == "blue" else 3) + e["scorer"]
    inset = action_inset(
        a.showcase / "trace-000.npz", start, e["shot"] * 0.04, e["winner"], receiver
    )
    clip = make_gif(movie, movie.with_suffix(".gif"), start, end - start, 12.5, inset)
    clip["native_zoom_crop_and_overlay_x"] = inset
    save(
        movie.with_suffix(".json"),
        dict(
            native_sha256=digest(movie),
            full_match_seconds=cfg["seconds"],
            excerpt=clip,
            source_start=start,
            source_end=end,
            play=e,
            seed=game["seed"],
            trace_delta=delta,
            speed=1,
        ),
    )
    print(result["identical_events"], delta, clip)


if __name__ == "__main__":
    main()
