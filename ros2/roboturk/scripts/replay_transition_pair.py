"""Record both teams on one matched start and verify both reruns against the original traces."""

import argparse
from pathlib import Path
import numpy as np
from train_recovery_transition import ROOT, WORLD, read, save, run_matches


def main():
    p = argparse.ArgumentParser()
    p.add_argument("qualification", type=Path)
    p.add_argument("baseline", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument(
        "--webots",
        type=Path,
        default=ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots",
    )
    a = p.parse_args()
    report = read(a.baseline / "comparison.json")
    examples = []
    for i, (before, after) in enumerate(
        zip(
            report["losses"]["previous"]["games"], report["losses"]["trained"]["games"], strict=True
        )
    ):
        if before["outcome"] == "goal" or after["outcome"] != "goal":
            continue
        goal = read(a.qualification / f"game-{i:03}.json")["conversions"][0]
        examples.append(
            (
                (
                    before["outcome"] == "opponent touch before first kick",
                    bool(goal["passes"]),
                    goal["dribble_m"] >= 0.18,
                    -goal["goal"],
                ),
                i,
            )
        )
    if not examples:
        raise ValueError("No newly converted matched start")
    _, i = max(examples)
    a.output.mkdir(parents=True, exist_ok=False)
    proof = {}
    for k, src, world in (
        ("previous", a.baseline, ROOT / "src/roboturk_ros/webots/worlds/ssl_shield_attack.wbt"),
        ("trained", a.qualification, WORLD),
    ):
        dst = (a.output / k).resolve()
        dst.mkdir()
        original = read(src / f"game-{i:03}.json")
        cfg = read(src / "launch.json") | dict(
            mode="demo", cases=1, first=original["seed"], record=True
        )
        save(dst / "launch.json", cfg)
        run_matches(dst, cfg, a.webots, world, True)
        played = read(dst / "game-000.json")
        with np.load(src / f"trace-{i:03}.npz") as first, np.load(dst / "trace-000.npz") as second:
            delta = float(abs(first["states"] - second["states"]).max())
        assert delta == 0 and all(
            original[x] == played[x] for x in ("events", "options", "conversions")
        )
        proof[k] = dict(
            seed=original["seed"],
            samples=cfg["seconds"] * 100,
            max_state_delta=delta,
            identical_options_events_goals=True,
        )
    save(
        a.output / "replay.json",
        dict(
            index=i,
            before=report["losses"]["previous"]["games"][i],
            play=read(a.qualification / f"game-{i:03}.json")["conversions"][0],
            proof=proof,
        ),
    )


if __name__ == "__main__":
    main()
