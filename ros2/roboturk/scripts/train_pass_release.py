"""Schedule pass-release training and paired qualification in native Webots."""

import argparse
import math
from pathlib import Path

import numpy as np
from compare_takeaways import all_games, trace
from takeaway_native import launch_cases
from train_counterpass import ROOT, WEBOTS, WORLD as INCUMBENT_WORLD, read, save, sha
from train_takeaway import summarize
from roboturk.open_play_audit import recoveries

DEST = ROOT / "policies/pass_release_v1"
WORLD = INCUMBENT_WORLD.with_name("ssl_pass_release.wbt")
CTRL = WORLD.parent.parent / "controllers/roboturk_pass_release/roboturk_pass_release.py"


def attempts(game):
    for chain in game["recoveries"]:
        if chain["kind"] != "takeaway":
            continue
        for p in game["passes"]:
            if p["team"] == chain["team"] and chain["sample"] <= 4 * p["tick"] <= chain["end"]:
                yield chain, p


def inspect_archive(plan):
    rows = []
    for g, f in all_games(ROOT / plan["archive"]):
        if g["blue_model"] != g["yellow_model"]:
            continue
        s = trace(f)
        for chain, p in attempts(g):
            d = next(v for v in g["takeaway_actions"] if v["sample"] == chain["sample"])
            assert p["receiver"] == d["recipient"]
            k = 4 * p["tick"]
            side = 3 * int(p["team"] == "yellow")
            receiver = side + p["receiver"]
            q = s[k - 1, 4:].reshape(6, 3)[:, :2]
            rivals = q[3 - side : 6 - side]
            target = np.array(d["target"])
            ball = s[k - 1, :2]
            stop = min(len(s), 4 * (p["end"] + 1))
            flight = s[k:stop]
            pose = flight[:, 4:].reshape(-1, 6, 3)[:, receiver]
            delta = flight[:, :2] - pose[:, :2]
            co, si = np.cos(pose[:, 2]), np.sin(pose[:, 2])
            mouth = (abs(delta[:, 0] * co + delta[:, 1] * si - 0.115) < 0.035) & (
                abs(delta[:, 1] * co - delta[:, 0] * si) < 0.045
            )
            rows.append(
                dict(
                    source=f.name,
                    seed=g["seed"],
                    sample=chain["sample"],
                    team=p["team"],
                    tick=p["tick"],
                    receiver=p["receiver"],
                    outcome=p["result"],
                    end=p["end"],
                    receiver_samples=int(mouth.sum()),
                    travel_m=float(np.linalg.norm(flight[-1, :2] - ball)),
                    length_m=float(np.linalg.norm(target - ball)),
                    rival_target_m=float(np.linalg.norm(rivals - target, axis=1).min()),
                    last_receiver_distance_m=float(np.linalg.norm(delta[-1])),
                    goal="goal" in chain,
                )
            )
    return rows


def branches(rows):
    groups = (
        [r for r in rows if r["outcome"] == "intercepted"],
        [r for r in rows if r["outcome"] == "complete"],
        [r for r in rows if r["outcome"] not in ("intercepted", "complete")],
    )
    selected = []
    for group, limit in zip(groups, (24, 24, 5), strict=True):
        for idx in np.linspace(0, len(group) - 1, min(len(group), limit)).round().astype(int):
            selected.append(group[idx])
    return sorted(selected, key=lambda r: (r["seed"], r["tick"]))


def verify_lock(folder):
    lock = read(folder / "freeze.json")
    for name, expected in lock.items():
        if sha(ROOT / name) != expected:
            raise ValueError(f"Frozen artifact differs: {name}")


def run(mode, folder, checkpoint, workers, resume):
    plan = read(DEST / "protocol.json")
    verify_lock(ROOT / "policies/counterpass_v1")
    base = read(ROOT / plan["baseline"])
    files = []
    if mode in ("train", "historical"):
        rows = inspect_archive(plan)
        files = (
            branches(rows)
            if mode == "train"
            else [r for r in rows if r["outcome"] == "intercepted"]
        )
    if mode == "train":
        candidates = [base | {"pass_release": {"action": action}} for action in (1, 2, 3)]
    elif mode == "dev":
        candidates = read(checkpoint)
    elif mode in ("control", "dev-control"):
        candidates = []
    else:
        candidates = [read(checkpoint)]
    if mode in ("evaluate", "historical", "control"):
        verify_lock(checkpoint.parent)
        assert (
            sha(checkpoint)
            == read(checkpoint.parent / "freeze.json")[str(checkpoint.relative_to(ROOT))]
        )
    jobs = []
    if files:
        for r in files:
            for j in range(1, len(candidates) + 1):
                b, y = (j, 0) if r["team"] == "blue" else (0, j)
                jobs.append(
                    (r["seed"], b, y, min(90, math.ceil(0.04 * r["tick"] + 12)), r["tick"], r)
                )
    elif mode == "dev-control":
        jobs.extend((plan["dev_first"] + i, 0, 0, 60, None, None) for i in range(6))
    elif mode == "dev":
        for i in range(6):
            for j in range(1, len(candidates) + 1):
                for b, y in ((j, j), (j, 0), (0, j)):
                    jobs.append((plan["dev_first"] + i, b, y, 60, None, None))
    else:
        for i in range(plan["fresh_starts"]):
            sides = (
                [(0, 0)]
                if mode == "control"
                else [(1, 1)] + ([(0, 1), (1, 0)] if i < plan["mixed_starts"] else [])
            )
            jobs.extend((plan["fresh_first"] + i, b, y, 90, None, None) for b, y in sides)
    world = INCUMBENT_WORLD if mode in ("control", "dev-control") else WORLD
    cfg = dict(
        root=str(ROOT),
        world=str(world.relative_to(ROOT)),
        models=[base, *candidates],
        schedule=[list(j[:3]) for j in jobs],
        cases=len(jobs),
        seconds=90,
        durations=[j[3] for j in jobs],
        release_ticks=[j[4] for j in jobs],
        references=[j[5] for j in jobs],
        press=True,
        finish_search=True,
        record=False,
        webots_sha256=sha(WEBOTS),
    )
    launch_cases(folder, cfg, WEBOTS, workers, resume)
    games = []
    for i, (g, f) in enumerate(all_games(folder)):
        s = trace(f)
        assert [g[k] for k in ("seed", "blue_model", "yellow_model")] == cfg["schedule"][i]
        assert len(s) == 100 * cfg["durations"][i] and g["recoveries"] == recoveries(s, g)
        if files:
            r = cfg["references"][i]
            old = trace(ROOT / plan["archive"] / r["source"])
            assert np.array_equal(s[: 4 * r["tick"]], old[: 4 * r["tick"]])
            assert (
                len(g["release_decisions"]) == 1 and g["release_decisions"][0]["tick"] == r["tick"]
            )
        games.append(g)
    assert len(games) == cfg["cases"]
    save(folder / "summary.json", summarize(games))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "mode",
        choices=["audit", "train", "dev", "dev-control", "control", "evaluate", "historical"],
    )
    p.add_argument("folder", type=Path)
    p.add_argument("--checkpoint", type=Path, default=DEST / "policy.json")
    p.add_argument("--workers", type=int, choices=range(1, 9), default=4)
    p.add_argument("--resume", action="store_true")
    a = p.parse_args()
    if a.mode == "audit":
        save(a.folder, inspect_archive(read(DEST / "protocol.json")))
    else:
        run(a.mode, a.folder.resolve(), a.checkpoint.resolve(), a.workers, a.resume)
