"""Replay pressured completed passes with alternative receiver motors on the same opening."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
import train_touch_planner as trainer


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("previous", type=Path)
    p.add_argument("models", type=Path)
    p.add_argument("folder", type=Path)
    p.add_argument("--webots", type=Path, default=trainer.WEBOTS)
    p.add_argument(
        "--all", action="store_true", help="Replay all previous openings as a regression panel"
    )
    args = p.parse_args()
    d = args.folder.resolve()
    d.mkdir(parents=True, exist_ok=False)
    trainer.WORLD = trainer.WORLD.with_name("ssl_clean_recovery.wbt")
    cfg = trainer.read(args.previous / "launch.json")
    models = trainer.read(args.models)
    if isinstance(models, dict):
        models = [models]
    rows = trainer.read(args.previous / "audit.json")["rows"]
    sources = [
        (i, r)
        for i, r in enumerate(rows)
        if r.get("pressure")
        and not r["qualified"]
        and not r["prepass_loss"]
        and r.get("pass_result") == "complete"
    ]
    if args.all:
        sources = list(enumerate(rows))
    schedule = [[r["seed"], a, a] for _, r in sources for a in range(len(models))]
    cfg.update(
        root=str(trainer.ROOT),
        models=models,
        schedule=schedule,
        cases=len(schedule),
        world=str(trainer.WORLD.relative_to(trainer.ROOT)),
        study=False,
        record=False,
    )
    assert trainer.sha(args.webots) == cfg["webots_sha256"]
    trainer.save(d / "launch.json", cfg)
    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(lambda i: trainer.isolated(d, i, args.webots), range(len(schedule))))
    report = []
    for i, (_, arm, _) in enumerate(schedule):
        j, old = sources[i // len(models)]
        a = np.load(args.previous / f"trace-{j:03}.npz")["states"]
        b = np.load(d / f"trace-{i:03}.npz")["states"]
        g = trainer.read(d / f"game-{i:03}.json")
        r = g["opening"]
        assert np.array_equal(a[0], b[0]) and trainer.touch_result(b, g, cfg["protocol"]) == r
        first = np.flatnonzero(np.any(a != b, axis=1))
        prefix = int(first[0]) if len(first) else len(a)
        report.append(
            dict(
                seed=old["seed"],
                arm=arm,
                same_through_release=prefix >= 4 * old["kick"]
                if old.get("kick") is not None
                else None,
                old_qualified=old["qualified"],
                prepass_loss=r["prepass_loss"],
                old_prepass_loss=old["prepass_loss"],
                identical_prefix_samples=prefix,
                old_outcome=old["outcome"],
                new_outcome=r["outcome"],
                qualified=r["qualified"],
            )
        )
    trainer.save(d / "diagnosis.json", report)
    print([(i, sum(v["qualified"] for v in report if v["arm"] == i)) for i in range(len(models))])


if __name__ == "__main__":
    main()
