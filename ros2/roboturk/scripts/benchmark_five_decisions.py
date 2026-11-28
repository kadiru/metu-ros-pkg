"""Compare decision controllers on matched native Webots starts with both colors."""

import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from audit_five_roles import audit, load_match
from controller_versions import load_catalog, source_build


ROOT = Path(__file__).resolve().parents[1]
TEAMS = ("blue", "yellow")


def play(output, seed, seconds, controllers):
    strategies = {
        team: dict(strategy="balanced", pass_preference=0, controller=controller)
        for team, controller in zip(TEAMS, controllers, strict=True)
    }
    subprocess.run(
        [
            sys.executable,
            "scripts/run_five_roles.py",
            str(output),
            "--seed",
            str(seed),
            "--seconds",
            str(seconds),
            "--team-strategies",
            json.dumps(strategies),
            "--open-play-policy",
            "policies/open_motion_v1/actor.json",
            "--trajectory-planner",
            "--errt-fallback",
            "--commit-waypoints",
            "--release-waypoint-early",
            "--crowded-fetch",
            "--tackle-policy",
            "policies/ball_side_tackle_v1/actor.json",
            "--tackle-selector",
            "--keeper-cover",
            "--keeper-lane",
            "--smooth-carry",
            "--receive-profile",
            "policies/five_receive_v1/policy.json",
            "--role-audit",
            "--possession-audit",
        ],
        cwd=ROOT,
        check=True,
    )
    game, states = load_match(output)
    report = audit(output)
    if game["seconds"] != seconds:
        raise ValueError(f"Incomplete match in {output}")
    receptions = Counter(game["keys"][e["passer"]][0] for e in report["confirmed_passes"])
    passes = Counter(game["keys"][e["actor"]][0] for e in game["events"] if e["event"] == "pass")
    first = game["frames"][0]["opening_velocities"]
    result = dict(
        score=game["score"],
        passes=[passes[t] for t in TEAMS],
        completed=[receptions[t] for t in TEAMS],
        contacts=game["robot_clearance"]["contact_episodes"],
    )
    return result, states[0], first


def main():
    catalog = load_catalog()
    installed = catalog["installed"]
    releases = catalog["releases"]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--seed", type=int, default=38728000)
    parser.add_argument("--starts", type=int, default=4)
    parser.add_argument("--seconds", type=int, default=25)
    parser.add_argument("--baseline", choices=installed, default=installed[0])
    parser.add_argument("--challenger", choices=installed, action="append")
    args = parser.parse_args()
    if args.starts < 1 or args.seconds < 1:
        parser.error("Use at least one start and a positive duration.")
    challengers = args.challenger or [key for key in installed if key != args.baseline]
    if args.baseline in challengers or len(set(challengers)) != len(challengers):
        parser.error("Choose distinct challenger versions with a separate baseline.")
    build = source_build()
    baseline_controller = releases[args.baseline]["implementation"]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    rows, totals = [], {releases[key]["implementation"]: Counter() for key in challengers}
    for seed in range(args.seed, args.seed + args.starts):
        baseline, initial, velocities = play(
            output / str(seed) / baseline_controller, seed, args.seconds, (baseline_controller,) * 2
        )
        for release in challengers:
            name = releases[release]["implementation"]
            for side, team in enumerate(TEAMS):
                opponents = [baseline_controller, baseline_controller]
                opponents[side] = name
                changed, state, speed = play(
                    output / str(seed) / f"{name}-{team}", seed, args.seconds, opponents
                )
                if not np.array_equal(initial, state) or velocities != speed:
                    raise ValueError(f"Different Webots opening for {seed}, {name}, {team}")
                rival = 1 - side
                diff = changed["score"][side] - changed["score"][rival]
                totals[name].update(
                    dict(
                        matches=1,
                        wins=int(diff > 0),
                        draws=int(diff == 0),
                        losses=int(diff < 0),
                        goals_for=changed["score"][side],
                        goals_against=changed["score"][rival],
                        baseline_goals_for=baseline["score"][side],
                        baseline_goals_against=baseline["score"][rival],
                        completed=changed["completed"][side],
                        passes=changed["passes"][side],
                        baseline_completed=baseline["completed"][side],
                        baseline_passes=baseline["passes"][side],
                        contacts=changed["contacts"],
                        baseline_contacts=baseline["contacts"],
                    )
                )
                rows.append(
                    dict(
                        seed=seed,
                        controller=name,
                        controller_version=release,
                        team=team,
                        baseline=baseline,
                        changed=changed,
                    )
                )
        print(f"Finished matched start {seed}", flush=True)
    report = dict(
        seconds=args.seconds, referee="local", totals=totals, pairs=rows,
        controller_versions=dict(
            build=build,
            baseline=args.baseline,
            releases={key: releases[key] for key in [args.baseline, *challengers]},
        ),
    )
    (output / "comparison.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(totals, indent=2))


if __name__ == "__main__":
    main()
