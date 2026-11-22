"""Find and film a verified pass goal beside the old finish from the identical steal."""

import argparse
import math
from pathlib import Path
import numpy as np
from audit_counterpass import pass_detail
from compare_takeaways import all_games, trace
from takeaway_native import launch_cases
from train_takeaway import run_matches
from train_counterpass import ROOT, WEBOTS, DEST, read, save, sha


def pressured_shot(g, r):
    return r["outcome"] in ("opponent touch", "ball out") and any(
        v["event"] == "kick"
        and v["team"] == r["team"]
        and v["speed"] == 4
        and r["sample"] <= 4 * v["tick"] <= r["end"]
        for v in g["events"]
    )


def find(folder, evaluation, control):
    prior = {g["seed"]: (g, f) for g, f in all_games(control)}
    eligible = []
    for g, f in all_games(evaluation):
        if (g["blue_model"], g["yellow_model"]) != (1, 1):
            continue
        states = trace(f)
        old, rf = prior[g["seed"]]
        before = trace(rf)
        for r in g["recoveries"]:
            if r["kind"] != "takeaway" or not r.get("passes"):
                continue
            detail = pass_detail(g, states, r)
            if not detail or not detail["verified_lead"] or not detail["open_at_arrival"]:
                continue
            through = 4 * (r["sample"] // 4 + 1)
            v = next(
                (
                    q
                    for q in old["recoveries"]
                    if q["sample"] == r["sample"]
                    and q["winner"] == r["winner"]
                    and q["kind"] == "takeaway"
                ),
                None,
            )
            direct = (
                v and pressured_shot(old, v) and np.array_equal(states[:through], before[:through])
            )
            eligible.append(
                dict(
                    seed=g["seed"],
                    slot=int(f.stem[-3:]),
                    sample=r["sample"],
                    trained=r,
                    detail=detail,
                    direct=bool(direct),
                    old_slot=int(rf.stem[-3:]),
                    old=v if direct else None,
                )
            )
    eligible.sort(key=lambda v: (not v["direct"], v["seed"], v["sample"]))
    if not eligible:
        raise ValueError("No verified open-runner lead goal in the supplied evaluation")
    folder.mkdir(parents=True, exist_ok=True)
    chosen = eligible[0]
    if chosen["direct"]:
        chosen.update(old_folder=str(control.resolve()), counterfactual=False)
    else:
        # Preserve all players' learned decisions until the one decision under test.
        # Compare only against the baseline finish at this physical state.
        candidates = eligible[:12]
        cfg = read(evaluation / "launch.json")
        cfg.update(
            cases=len(candidates),
            schedule=[cfg["schedule"][v["slot"]] for v in candidates],
            decision_starts=[None] * len(candidates),
            references=[None] * len(candidates),
            durations=[min(90, math.ceil(v["sample"] / 100) + 18) for v in candidates],
            incumbent_samples=[v["sample"] for v in candidates],
        )
        search = folder / "probes"
        launch_cases(search.resolve(), cfg, WEBOTS, 4, search.exists())
        for i, v in enumerate(candidates):
            g = read(search / f"game-{i:03}.json")
            old = next(q for q in g["recoveries"] if q["sample"] == v["sample"])
            a = trace(search / f"game-{i:03}.json")
            b = trace(evaluation / f"game-{v['slot']:03}.json")
            through = 4 * (v["sample"] // 4 + 1)
            assert np.array_equal(a[:through], b[:through])
            if pressured_shot(g, old):
                chosen = v | dict(
                    old=old, old_slot=i, old_folder=str(search.resolve()), counterfactual=True
                )
                break
        else:
            raise ValueError(
                "No verified incumbent shot loss among these candidate counterfactuals"
            )
    chosen["new_folder"] = str(evaluation.resolve())
    chosen["identical_samples"] = 4 * (chosen["sample"] // 4 + 1)
    save(folder / "selected.json", chosen)


def record(root, arm):
    case = read(root / "selected.json")
    prev = arm == "previous"
    src = Path(case["old_folder" if prev else "new_folder"])
    idx = case["old_slot" if prev else "slot"]
    cfg = read(src / "launch.json")
    for k in ("schedule", "decision_starts", "references", "durations", "incumbent_samples"):
        if k in cfg:
            cfg[k] = [cfg[k][idx]]
    cfg.update(root=str(ROOT), cases=1, record=True)
    folder = (root / arm).resolve()
    folder.mkdir(parents=True, exist_ok=True)
    save(folder / "launch.json", cfg)
    if not (folder / "match.mp4").exists():
        run_matches(folder, cfg, WEBOTS, ROOT / cfg["world"], True)
    actual = trace(folder / "game-000.json")
    expected = trace(src / f"game-{idx:03}.json")
    assert np.array_equal(actual, expected)
    g = read(folder / "game-000.json")
    assert g == read(src / f"game-{idx:03}.json")
    save(
        folder / "replay.json",
        dict(
            seed=g["seed"],
            trained_moment=case["trained"],
            model=int(not prev),
            exact_trajectory=True,
            exact_events=True,
            physical_samples=len(actual),
            same_recovery_samples=case["identical_samples"],
            distribution_choice=case["detail"],
            incumbent_only_at_this_decision=case["counterfactual"],
        ),
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=["find", "previous", "trained"])
    p.add_argument("directory", type=Path)
    p.add_argument("--evaluation", type=Path)
    p.add_argument("--control", type=Path)
    a = p.parse_args()
    assert all(sha(ROOT / k) == v for k, v in read(DEST / "freeze.json").items())
    if a.mode == "find":
        find(a.directory, a.evaluation, a.control)
    else:
        record(a.directory, a.mode)
