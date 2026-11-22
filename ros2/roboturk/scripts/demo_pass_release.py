"""Select and independently film a full match and a matched intercepted possession."""

import argparse
from pathlib import Path
import numpy as np
from audit_pass_release import review_outcomes
from compare_takeaways import all_games, trace
from train_pass_release import DEST, ROOT, WEBOTS, attempts, read, save, verify_lock
from train_takeaway import run_matches


def select_case(out, control, new, historical):
    chosen = []
    fresh = []
    full_choices = {}
    base = {g["seed"]: (g, f) for g, f in all_games(control)}
    archive = ROOT / read(DEST / "protocol.json")["archive"]
    for g, f in all_games(new) + all_games(historical):
        hist = f.parent == historical
        if not hist and g["blue_model"] != g["yellow_model"]:
            continue
        s = trace(f)
        if hist:
            idx = int(f.stem.split("-")[-1])
            ref = read(historical / "launch.json")["references"][idx]
            rf = archive / ref["source"]
            old = read(rf)
        else:
            old, rf = base[g["seed"]]
        reviews = review_outcomes(g)
        if not hist:
            clips = {}
            for d in reviews:
                a = d["action"]
                if a == 0:
                    continue
                valid = (
                    (
                        d["release_tick"] is not None
                        and d["released_to"] == (d["intended"] if a == 1 else d["alternative"])
                    )
                    if a in (1, 2)
                    else d["goal"] is not None
                )
                if not valid:
                    continue
                quality = (d["completed"], d["goal"] is not None, -d["tick"])
                if a not in clips or quality > clips[a][0]:
                    clips[a] = (quality, d)
            full_choices[int(f.stem[-3:])] = [item[1] for _, item in sorted(clips.items())]
        for d in reviews:
            if d["action"] == 0 or d["goal"] is None:
                continue
            r = next(
                r for r in g["recoveries"] if r["sample"] == d["sample"] and r["team"] == d["team"]
            )
            pair = next(
                (
                    (chain, p)
                    for chain, p in attempts(old)
                    if p["tick"] == d["tick"]
                    and p["team"] == d["team"]
                    and p["result"] == "intercepted"
                ),
                None,
            )
            same = pair is not None and np.array_equal(
                s[: 4 * d["tick"]], trace(rf)[: 4 * d["tick"]]
            )
            rank = (
                not (d["action"] in (1, 2) and d["completed"]),
                not d["completed"],
                hist,
                g["seed"],
                d["tick"],
            )
            entry = dict(
                seed=g["seed"],
                source="historical branch" if hist else "fresh mirrors",
                trained=r,
                release_decision=d,
                new_folder=str(f.parent.resolve()),
                slot=int(f.stem[-3:]),
                sample=r["sample"],
                identical_samples=4 * d["tick"],
                counterfactual=hist,
            )
            if not hist:
                fresh.append((rank, entry))
            if same:
                old_chain, p = pair
                entry = entry | dict(
                    old_folder=str(rf.parent.resolve()),
                    old_slot=int(rf.stem[-3:]),
                    old=old_chain,
                    old_pass=p,
                )
                chosen.append((rank, entry))
    if not chosen or not fresh:
        raise ValueError("No matched interception-to-goal or fresh scoring decision found")
    # Prefer an actual switched or delayed pass and goal, then an independent fresh pair.
    best = min(chosen, key=lambda v: v[0])[1]

    def coverage(item):
        rank, entry = item
        clips = full_choices[entry["slot"]]
        return (
            -len(clips),
            -sum(c["completed"] for c in clips),
            -sum(c["goal"] is not None for c in clips),
            rank,
        )

    full = min(fresh, key=coverage)[1]
    full = full | dict(clips=full_choices[full["slot"]])
    out.mkdir(parents=True, exist_ok=True)
    save(out / "selected.json", best)
    save(out / "full.json", full)


def record(folder, mode):
    case = read(folder / ("full.json" if mode == "full" else "selected.json"))
    old = mode == "previous"
    src = Path(case["old_folder" if old else "new_folder"])
    idx = case["old_slot" if old else "slot"]
    cfg = read(src / "launch.json")
    for key in (
        "schedule",
        "durations",
        "references",
        "release_ticks",
        "decision_starts",
        "incumbent_samples",
    ):
        if key in cfg:
            cfg[key] = [cfg[key][idx]]
    cfg.update(root=str(ROOT), record=True, cases=1)
    dst = (folder / mode).resolve()
    dst.mkdir(parents=True, exist_ok=True)
    save(dst / "launch.json", cfg)
    if not (dst / "match.mp4").exists():
        run_matches(dst, cfg, WEBOTS, ROOT / cfg["world"], True)
    s = trace(dst / "game-000.json")
    assert np.array_equal(s, trace(src / f"game-{idx:03}.json"))
    g = read(dst / "game-000.json")
    assert g == read(src / f"game-{idx:03}.json")
    save(
        dst / "replay.json",
        dict(
            seed=g["seed"],
            trained_moment=case["trained"],
            model=int(not old),
            exact_events=True,
            exact_trajectory=True,
            physical_samples=len(s),
            decision=case["release_decision"],
            source=case["source"],
        ),
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=["find", "previous", "trained", "full"])
    p.add_argument("folder", type=Path)
    p.add_argument("--control", type=Path)
    p.add_argument("--new", type=Path)
    p.add_argument("--historical", type=Path)
    a = p.parse_args()
    verify_lock(DEST)
    if a.mode == "find":
        select_case(a.folder, a.control, a.new, a.historical)
    else:
        record(a.folder, a.mode)
