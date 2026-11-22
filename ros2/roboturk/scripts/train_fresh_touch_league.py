"""Select first-touch teams using matched cold Webots launches for every training game."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import gzip
import json
from pathlib import Path
import numpy as np

import train_touch_planner as trainer


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("models", type=Path)
    p.add_argument("folder", type=Path)
    p.add_argument("--offset", type=int, required=True)
    p.add_argument("--cases", type=int, default=24)
    p.add_argument("--workers", type=int, default=3, choices=(1, 2, 3, 4))
    p.add_argument("--webots", type=Path, default=trainer.WEBOTS)
    args = p.parse_args()
    if not 0 <= args.offset < 90000 or not 2 <= args.cases <= 200:
        p.error("Invalid training panel")
    trainer.WORLD = trainer.WORLD.with_name("ssl_clean_recovery.wbt")
    models = trainer.read(args.models)
    protocol = trainer.read(trainer.R / "protocol.json")
    start = protocol["train_first"] + args.offset
    schedule = [[start + i, j, j] for i in range(args.cases) for j in range(len(models))]
    d = args.folder.resolve()
    d.mkdir(parents=True, exist_ok=False)
    cfg = dict(
        root=str(trainer.ROOT),
        models=models,
        schedule=schedule,
        cases=len(schedule),
        protocol=protocol,
        seconds=18,
        press=True,
        study=False,
        record=False,
        world=str(trainer.WORLD.relative_to(trainer.ROOT)),
        webots_sha256=trainer.sha(args.webots),
        independent_processes=True,
    )
    trainer.save(d / "launch.json", cfg)
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        list(pool.map(lambda i: trainer.isolated(d, i, args.webots), range(len(schedule))))
    games = []
    rows = [[] for _ in models]
    for i, (seed, j, _) in enumerate(schedule):
        g = trainer.read(d / f"game-{i:03}.json")
        s = np.load(d / f"trace-{i:03}.npz")["states"]
        assert (
            len(s) == 1800
            and g["seed"] == seed
            and trainer.touch_result(s, g, protocol) == g["opening"]
        )
        if j == 0:
            first = s[0]
        assert np.array_equal(first, s[0]), (seed, j)
        games.append(g)
        rows[j].append(g["opening"])
    stats = [trainer.panel(group) for group in rows]
    best = max(
        range(len(models)),
        key=lambda j: (stats[j]["goals"] - 2 * stats[j]["prepass_losses"], stats[j]["goals"]),
    )
    trainer.save(d / "returns.json", stats)
    trainer.save(d / "policy.json", models[best])
    data = dict(
        config={k: v for k, v in cfg.items() if k != "root"},
        returns=stats,
        selected=best,
        matching_initial_frames=args.cases,
        games=games,
    )
    (d / "training.json.gz").write_bytes(
        gzip.compress(json.dumps(data, separators=(",", ":")).encode(), mtime=0)
    )
    print([(s["goals"], s["prepass_losses"]) for s in stats], flush=True)


if __name__ == "__main__":
    main()
