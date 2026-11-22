"""Train a clean-takeaway option policy and evaluate it in ordinary native matches."""

import argparse
from takeaway_native import launch_cases
import gzip
import json
import math
from pathlib import Path

import numpy as np
from train_recovery_touch import ROOT, WEBOTS, read, save, sha, run_matches
from roboturk.open_play_audit import recoveries
from roboturk.takeaway import MODES
from fit_takeaway import fit
from roboturk.takeaway_outlet import EXTRA

DEST = ROOT / "policies/takeaway_v1"
OLD = ROOT / "policies/open_play_v1"
WORLD = ROOT / "src/roboturk_ros/webots/worlds/ssl_takeaway.wbt"
CTRL = WORLD.parent.parent / "controllers/roboturk_takeaway/roboturk_takeaway.py"


def source_rows(plan):
    archive = json.loads(gzip.decompress((ROOT / plan["training_archive"]).read_bytes()))
    cases = []
    for i, g in enumerate(archive["games"]):
        for r in g["recoveries"]:
            if r["kind"] == "takeaway" and g["seconds"] - r["sample"] / 100 >= 12:
                cases.append((i, g, r))
                break
    # Spread the branches over the archive's kickoff seeds and opponent combinations.
    assert len(cases) >= plan["training_cases"]
    ids = np.linspace(0, len(cases) - 1, plan["training_cases"]).round().astype(int)
    return archive["config"]["models"], [cases[i] for i in ids]


def historical():
    cases = []
    for f in sorted((OLD / "evaluation").glob("game-*.json")):
        g = read(f)
        if (g["blue_model"], g["yellow_model"]) != (1, 1):
            continue
        cases.extend((f, g, r) for r in g["recoveries"] if r["kind"] == "takeaway")
    assert len(cases) == 3
    return cases


def summarize(games):
    def stats(selected, team_index):
        rows = []
        scores = 0
        n = 0
        for g in selected:
            teams = [t for t in ("blue", "yellow") if g[t + "_model"] == team_index]
            if not teams:
                continue
            n += 1
            rows.extend(r for r in g["recoveries"] if r["team"] in teams)
            scores += sum(g["score"].get(t, 0) for t in teams)
        clean = [r for r in rows if r["kind"] == "takeaway"]
        return dict(
            matches=n,
            takeaways=len(clean),
            takeaway_goals=sum("goal" in r for r in clean),
            recovery_goals=sum("goal" in r for r in rows),
            recoveries=len(rows),
            all_goals=scores,
        )

    mirror = [g for g in games if g["blue_model"] == g["yellow_model"]]
    mixed = [g for g in games if g["blue_model"] != g["yellow_model"]]
    return {
        n: [stats(group, j) for j in (0, 1)] for n, group in [("mirror", mirror), ("mixed", mixed)]
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "mode",
        choices=[
            "train",
            "extend",
            "control",
            "historical",
            "historical-early",
            "evaluate",
            "record",
        ],
    )
    parser.add_argument("folder", type=Path)
    parser.add_argument("--workers", type=int, choices=range(1, 5), default=3)
    parser.add_argument("--webots", type=Path, default=WEBOTS)
    parser.add_argument("--from-evaluation", type=Path)
    parser.add_argument("--previous", action="store_true")
    parser.add_argument(
        "--calibrated", action="store_true", help="Use the separate historical fine-tune"
    )
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    if args.calibrated and args.mode not in (
        "historical",
        "historical-early",
        "evaluate",
        "record",
    ):
        parser.error("The calibrated policy is available for qualification and film")
    checkpoint = "calibrated.json" if args.calibrated else "policy.json"
    plan = read(DEST / "protocol.json")
    base = read(ROOT / plan["baseline"])
    for lock in [
        OLD / "freeze.json",
        *([DEST / "calibration-freeze.json"] if args.calibrated else []),
        *(
            [DEST / "freeze.json"]
            if args.mode in ("historical", "historical-early", "evaluate", "record")
            else []
        ),
    ]:
        for name, digest in read(lock).items():
            if sha(ROOT / name) != digest:
                raise ValueError(f"Freeze mismatch in {name}")
    schedule = []
    starts = []
    durations = []
    refs = []
    models = []
    nsec = plan["seconds"]

    def add(seed, blue, yellow, target=None, seconds=nsec, reference=None):
        schedule.append([seed, blue, yellow])
        starts.append(target)
        durations.append(seconds)
        refs.append(reference)

    def model(m):
        if m not in models:
            models.append(m)
        return models.index(m)

    if args.mode in ("train", "extend"):
        original, cases = source_rows(plan)
        for i, g, r in cases:
            actions = (
                range(len(MODES))
                if args.mode == "train"
                else range(len(MODES), len(MODES) + len(EXTRA))
            )
            for action in actions:
                teams = [original[g[t + "_model"]] for t in ("blue", "yellow")]
                j = int(r["team"] == "yellow")
                teams[j] = teams[j] | dict(takeaway_tree=dict(action=action))
                add(
                    g["seed"],
                    *map(model, teams),
                    r["sample"],
                    min(
                        g["seconds"],
                        math.ceil(r["sample"] / 100) + plan["training_followup_seconds"],
                    ),
                    i,
                )
    elif args.mode in ("historical", "historical-early"):
        models = [base, read(DEST / checkpoint)]
        for f, g, r in historical():
            for a in (0, 1):
                pair = (a, 0) if r["team"] == "blue" else (0, a)
                add(
                    g["seed"],
                    *pair,
                    r["sample"] if args.mode == "historical" else None,
                    reference=f.name,
                )
    else:
        models = [base] if args.mode == "control" else [base, read(DEST / checkpoint)]
        for i in range(plan["starts"]):
            seed = plan["fresh_first"] + i
            pairs = (
                [(0, 0)]
                if args.mode == "control"
                else [(1, 1)] + ([(0, 1), (1, 0)] if i < plan["mixed_starts"] else [])
            )
            for a, b in pairs:
                add(seed, a, b)
    source = args.from_evaluation
    if args.mode == "record":
        if not source:
            parser.error("Supply the evaluated games")
        choices = []
        for i, (seed, a, b) in enumerate(schedule):
            if (a, b) != (1, 1):
                continue
            for r in read(source / f"game-{i:03}.json")["recoveries"]:
                if r["kind"] == "takeaway" and "goal" in r:
                    choices.append((seed, r["sample"], i, r))
        if not choices:
            raise ValueError("No fresh clean-takeaway goal")
        seed, _, slot, moment = min(choices, key=lambda item: item[:2])
        if args.previous:
            source = source.parent / "control"
            slot = read(source / "launch.json")["schedule"].index([seed, 0, 0])
        schedule = [[seed, 0, 0] if args.previous else [seed, 1, 1]]
        starts = [None]
        durations = [nsec]
        refs = [None]
    d = args.folder.resolve()
    cfg = dict(
        root=str(ROOT),
        models=models,
        schedule=schedule,
        cases=len(schedule),
        seconds=nsec,
        durations=durations,
        starts=starts,
        press_from=(
            [100000] * len(schedule)
            if args.mode == "historical"
            else [(r["sample"] // 4 - 25) * 4 for _, _, r in historical() for _ in (0, 1)]
            if args.mode == "historical-early"
            else [0] * len(schedule)
        ),
        references=refs,
        press=True,
        record=args.mode == "record",
        **({"finish_search": True} if args.calibrated else {}),
        world=str(WORLD.relative_to(ROOT)),
        webots_sha256=sha(args.webots),
    )
    if args.mode == "record":
        d.mkdir(parents=True, exist_ok=False)
        save(d / "launch.json", cfg)
        run_matches(d, cfg, args.webots, WORLD, True)
        actual = np.load(d / "trace-000.npz")["states"]
        assert np.array_equal(actual, np.load(source / f"trace-{slot:03}.npz")["states"])
        assert read(d / "game-000.json") == read(source / f"game-{slot:03}.json")
        save(
            d / "replay.json",
            dict(
                seed=seed,
                trained_moment=moment,
                model=schedule[0][1],
                exact_trajectory=True,
                exact_events=True,
                physical_samples=len(actual),
            ),
        )
        return
    launch_cases(d, cfg, args.webots, args.workers, args.resume)
    games = []
    for i, (seed, b, y) in enumerate(schedule):
        g = read(d / f"game-{i:03}.json")
        s = np.load(d / f"trace-{i:03}.npz")["states"]
        assert len(s) == 100 * durations[i] and (seed, b, y) == (
            g["seed"],
            g["blue_model"],
            g["yellow_model"],
        )
        assert recoveries(s, g) == g["recoveries"]
        games.append(g)
    if args.mode == "train":
        fit(d, games, starts, refs, base)
    elif args.mode in ("historical", "historical-early"):
        outcomes = []
        for i, g in enumerate(games):
            source = OLD / "evaluation"
            name = refs[i]
            old = read(source / name)
            orig = np.load(source / name.replace("game", "trace").replace("json", "npz"))["states"]
            s = np.load(d / f"trace-{i:03}.npz")["states"]
            target = historical()[i // 2][2]
            through = (
                4 * (starts[i] // 4 + 1) if args.mode == "historical" else cfg["press_from"][i]
            )
            assert np.array_equal(orig[:through], s[:through])
            if i % 2 == 0:
                assert np.array_equal(orig, s) and all(
                    g[k] == v for k, v in old.items() if k not in ("blue_model", "yellow_model")
                )
            if args.mode == "historical":
                row = next(
                    r
                    for r in g["recoveries"]
                    if r["sample"] == starts[i] and r["kind"] == "takeaway"
                )
            else:
                nearby = [
                    r
                    for r in g["recoveries"]
                    if r["kind"] == "takeaway"
                    and r["winner"] == target["winner"]
                    and through <= r["sample"] <= target["sample"] + 300
                ]
                row = (
                    min(nearby, key=lambda r: abs(r["sample"] - target["sample"]))
                    if nearby
                    else dict(outcome="original clean takeover absent")
                )
            outcomes.append(
                dict(
                    seed=g["seed"],
                    role="trained" if i % 2 else "previous",
                    identical_prefix_samples=through,
                    **row,
                )
            )
        save(d / "summary.json", outcomes)
    else:
        save(d / "summary.json", summarize(games))


if __name__ == "__main__":
    main()
