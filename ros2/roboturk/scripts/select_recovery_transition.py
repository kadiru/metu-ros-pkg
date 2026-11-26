"""Reproduce the policy-search selection and bind its weights to the native training records."""

import argparse
import gzip
import json
import shutil
from pathlib import Path
from train_recovery_transition import R, ROOT, digest, read, save, sources


def ranking(stats):
    # Goals dominate. A tie favors a policy that also scored from completed passes.
    return sorted(stats, key=lambda s: (s["goals"], s["passes"], s["carries"]), reverse=True)


def refine(folder, dest):
    cfg = read(folder / "launch.json")
    ranked = ranking(read(folder / "returns.json"))
    best, second = [cfg["candidates"][s["index"]] for s in ranked[:2]]
    variants = [
        best,
        second,
        best | dict(pass_bias=-0.03, shot_bias=best["shot_bias"] + 0.10),
        best | dict(pass_bias=0.26, carry_bias=best["carry_bias"] + 0.10),
        best | dict(pass_bias=-0.22, tracking=0.38, centering=19.0, speed=1.15),
        second | dict(pass_bias=0.06, tracking=0.42, centering=11.0, escape=0.66),
    ]
    save(dest, variants)


def verify_training(file):
    with gzip.open(file, "rt") as f:
        archive = json.load(f)
    cfg = archive["launch"]
    arms = cfg["candidates"]
    games = archive["games"]
    assert cfg["mode"] == "motors" and len(games) == cfg["cases"]
    scores = [dict(index=j, games=0, goals=0, passes=0, carries=0) for j in range(len(arms))]
    for i, g in enumerate(games):
        layout = i // len(arms)
        j = (i + layout) % len(arms)
        assert g["seed"] == cfg["first"] + layout and g["control"] == j
        m = scores[j]
        m["games"] += 1
        e = (g["conversions"] or [{}])[0]
        if "goal" in e:
            m["goals"] += 1
            m["passes"] += int(bool(e["passes"]))
            m["carries"] += int(e["dribble_m"] >= 0.18)
    assert scores == archive["returns"]
    return arms[ranking(scores)[0]["index"]], scores


def main():
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="action", required=True)
    ref = sub.add_parser("refine")
    ref.add_argument("folder", type=Path)
    ref.add_argument("dest", type=Path)
    freeze = sub.add_parser("freeze")
    freeze.add_argument("folder", type=Path)
    freeze.add_argument("version", type=int)
    v = sub.add_parser("verify")
    v.add_argument("version", type=int)
    a = p.parse_args()
    if a.action == "refine":
        refine(a.folder, a.dest)
        return
    frozen = R / f"freeze-{a.version}.json"
    candidate = R / f"candidate-{a.version}.json"
    archive = R / f"training-{a.version}.json.gz"
    if a.action == "freeze":
        assert not frozen.exists() and not candidate.exists()
        winner, stats = verify_training(a.folder / "training.json.gz")
        assert winner == read(a.folder / "policy.json")
        shutil.copyfile(a.folder / "training.json.gz", archive)
        save(candidate, winner)
        files = [*sources(), Path(__file__).resolve(), archive, candidate]
        save(frozen, {str(f.relative_to(ROOT)): digest(f) for f in files})
        print(stats)
    else:
        for name, sha in read(frozen).items():
            assert digest(ROOT / name) == sha, name
        winner, stats = verify_training(archive)
        assert winner == read(candidate)
        print(stats)


if __name__ == "__main__":
    main()
