"""Train and compare open-play transition policies in independent native Webots matches."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np

import train_touch_planner as native
from train_recovery_touch import ROOT, WEBOTS, read, save, sha, run_matches
from roboturk.open_play_audit import recoveries

DEST = ROOT / "policies/open_play_v1"
WORLD = ROOT / "src/roboturk_ros/webots/worlds/ssl_open_play.wbt"
CTRL = WORLD.parent.parent / "controllers/roboturk_open_play/roboturk_open_play.py"


def population(base):
    # Optimize both the pressure off ball and the existing option values on ball.
    return [base] + [
        base
        | dict(
            counter_press=speed,
            anticipate=lead,
            bias=bias,
            block=block,
            advance=advance,
            shot_clear=clear,
        )
        for speed, lead, bias, block, advance, clear in [
            (0.88, 0.16, 0.70, 2.6, 0.93, 0.105),
            (1.20, 0.32, 0.18, 3.8, 1.00, 0.095),
            (1.12, 0.22, 0.98, 2.1, 0.78, 0.12),
        ]
    ]


def totals(games, model=None):
    matches = recover = goals = late = team_games = all_goals = 0
    for game in games:
        colors = [t for t in ("blue", "yellow") if model is None or game[t + "_model"] == model]
        if not colors:
            continue
        matches += 1
        team_games += len(colors)
        all_goals += sum(game["score"].get(t, 0) for t in colors)
        for r in game["recoveries"]:
            if r["team"] in colors:
                recover += 1
                goals += int("goal" in r)
                late += int(r.get("goal", -1) >= 4500)
    return dict(
        games=matches,
        team_games=team_games,
        recoveries=recover,
        goals=goals,
        all_goals=all_goals,
        goals_after_45s=late,
        conversion=goals / max(1, recover),
        goals_per_game=goals / max(1, matches),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["train", "freeze", "evaluate", "record"])
    parser.add_argument("folder", type=Path)
    parser.add_argument("--workers", type=int, choices=(1, 2, 3, 4), default=3)
    parser.add_argument("--webots", type=Path, default=WEBOTS)
    parser.add_argument("--from-evaluation", type=Path)
    parser.add_argument(
        "--previous", action="store_true", help="Film the matched previous-team game"
    )
    parser.add_argument("--audit-existing-training", action="store_true")
    args = parser.parse_args()
    if args.audit_existing_training and args.mode != "train":
        parser.error("Only complete training schedules can be re-audited")
    d = args.folder.resolve()
    plan = read(DEST / "protocol.json")
    prior = read(ROOT / plan["previous"])
    freeze = DEST / "freeze.json"
    policy = DEST / "policy.json"
    native.WORLD = WORLD
    if args.mode == "freeze":
        if freeze.exists() or policy.exists():
            parser.error("Checkpoint already frozen")
        save(policy, read(d / "selected.json"))
        files = [
            WORLD,
            CTRL,
            Path(__file__).resolve(),
            DEST / "protocol.json",
            policy,
            ROOT / plan["previous"],
            ROOT / "src/roboturk_core/roboturk/open_play.py",
            ROOT / "src/roboturk_core/roboturk/open_play_audit.py",
        ]
        save(freeze, {str(p.relative_to(ROOT)): sha(p) for p in files})
        return
    for manifest in [
        ROOT / "policies/recovery_touch_v1/planned-freeze-9.json",
        *([freeze] if args.mode != "train" else []),
    ]:
        for name, digest in read(manifest).items():
            if sha(ROOT / name) != digest:
                raise ValueError(f"Frozen dependency changed: {name}")
    if args.mode == "train":
        models = population(prior)
        schedule = []
        n = len(models)
        for i in range(plan["train_starts"]):
            seed = plan["train_first"] + i
            for j in range(n):
                schedule.append([seed, j, j])
                schedule.append([seed, j, (j + 1 + i % (n - 1)) % n])
        seconds = plan["training_seconds"]
    else:
        models = [prior, read(policy)]
        seconds = plan["match_seconds"]
        schedule = []
        for i in range(plan["test_starts"]):
            seed = plan["test_first"] + i
            pairs = [(0, 0), (1, 1)] + ([(0, 1), (1, 0)] if i < plan["mixed_starts"] else [])
            schedule.extend([seed, a, b] for a, b in pairs)
    if args.mode == "record":
        if not args.from_evaluation:
            parser.error("Recording requires the evaluated games")
        source = args.from_evaluation.resolve()
        ranking = []
        for i, (seed, a, b) in enumerate(schedule):
            if (a, b) != (1, 1):
                continue
            for row in read(source / f"game-{i:03}.json")["recoveries"]:
                if "goal" in row:
                    ranking.append(
                        (
                            (bool(row["passes"]), row["sample"] >= 4500, -seed, -row["sample"]),
                            i,
                            row,
                        )
                    )
        if not ranking:
            raise ValueError("No verified goal to replay")
        _, i, moment = max(ranking, key=lambda item: item[0])
        seed = schedule[i][0]
        if args.previous:
            i = schedule.index([seed, 0, 0])
        schedule = [schedule[i]]
    d.mkdir(parents=True, exist_ok=args.audit_existing_training)
    cfg = dict(
        root=str(ROOT),
        models=models,
        schedule=schedule,
        seconds=seconds,
        cases=len(schedule),
        press=True,
        record=args.mode == "record",
        world=str(WORLD.relative_to(ROOT)),
        webots_sha256=sha(args.webots),
    )
    if args.audit_existing_training:
        assert read(d / "launch.json") == cfg
    else:
        save(d / "launch.json", cfg)
    if args.mode == "record":
        run_matches(d, cfg, args.webots, WORLD, True)
        expected = source / f"trace-{i:03}.npz"
        assert np.array_equal(np.load(expected)["states"], np.load(d / "trace-000.npz")["states"])
        assert read(source / f"game-{i:03}.json") == read(d / "game-000.json")
        save(
            d / "replay.json",
            dict(
                seed=schedule[0][0],
                trained_moment=moment,
                model=schedule[0][1],
                physical_samples=seconds * 100,
                exact_events=True,
                exact_trajectory=True,
            ),
        )
        return
    if not args.audit_existing_training:
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            list(pool.map(lambda i: native.isolated(d, i, args.webots), range(len(schedule))))
    games, first = [], {}
    for i, (seed, a, b) in enumerate(schedule):
        game = read(d / f"game-{i:03}.json")
        states = np.load(d / f"trace-{i:03}.npz")["states"]
        assert len(states) == 100 * seconds and game["recoveries"] == recoveries(states, game)
        assert (game["seed"], game["blue_model"], game["yellow_model"]) == (seed, a, b)
        # The first trajectory sample already contains 10 ms of policy-dependent motion.
        if "initial_world" in game or not args.audit_existing_training:
            initial = game["initial_world"]
            assert first.setdefault(seed, initial) == initial, seed
        games.append(game)
    if args.mode == "train":
        stats = [totals(games, j) for j in range(len(models))]
        best = max(
            range(1, len(models)),
            key=lambda j: stats[j]["goals"] / stats[j]["team_games"] + stats[j]["conversion"],
        )
        save(d / "selected.json", models[best])
        result = dict(selected=best, candidates=stats)
    else:
        mirror = [g for g in games if g["blue_model"] == g["yellow_model"]]
        mixed = [g for g in games if g["blue_model"] != g["yellow_model"]]
        result = dict(
            matched_starts=len(first),
            mirror=[totals(mirror, j) for j in range(2)],
            head_to_head=[totals(mixed, j) for j in range(2)],
            contacts=sum(g["stats"]["contacts"] for g in games),
            exits=sum(g["stats"]["exits"] for g in games),
        )
    save(d / "summary.json", result)
    print(result, flush=True)


if __name__ == "__main__":
    main()
