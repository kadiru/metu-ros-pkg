"""Fit action policies on matched native Webots returns and freeze a league winner."""

import argparse
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import numpy as np
from roboturk.distribution_value import design, decision
from train_counterpass import ROOT, DEST, OLD, WORLD, CTRL, read, save, sha


def archive(folder, name):
    cfg = read(folder / "launch.json")
    cfg.pop("root")
    games = [read(folder / f"game-{i:03}.json") for i in range(cfg["cases"])]
    (DEST / (name + ".json.gz")).write_bytes(
        gzip.compress(
            json.dumps(dict(config=cfg, games=games), separators=(",", ":")).encode(), mtime=0
        )
    )


def rows(folder):
    cfg = read(folder / "launch.json")
    data = []
    for i in range(0, cfg["cases"], 3):
        name = cfg["references"][i]
        sample = cfg["decision_starts"][i]
        folder0 = OLD / "evaluation"
        base = read(folder0 / name)
        orig = np.load(folder0 / name.replace("game", "trace").replace("json", "npz"))["states"]
        through = 4 * (sample // 4 + 1)
        returns = []
        passed = []
        outcomes = []
        obs = None
        for j in range(4):
            g = read(folder / f"game-{i + j - 1:03}.json") if j else base
            r = next(
                v for v in g["recoveries"] if v["sample"] == sample and v["kind"] == "takeaway"
            )
            if j:
                s = np.load(folder / f"trace-{i + j - 1:03}.npz")["states"]
                assert g["initial_world"] == base["initial_world"] and np.array_equal(
                    orig[:through], s[:through]
                )
                options = [v for v in g["takeaway_actions"] if "distribution" in v]
                if options:
                    (c,) = options
                    assert c["distribution"] == j and c["sample"] == sample
                    if obs is not None:
                        assert c["distribution_obs"] == obs
                    obs = c["distribution_obs"]
            returns.append(int("goal" in r))
            passed.append(bool(r.get("passes")))
            outcomes.append(r["outcome"])
        data.append(
            dict(
                seed=base["seed"],
                source=name,
                sample=sample,
                index=i,
                x=obs,
                returns=returns,
                passed=passed,
                outcomes=outcomes,
                prefix_sha256=hashlib.sha256(orig[:through].tobytes()).hexdigest(),
            )
        )
    return data


def tree_fit(data, depth, minimum):
    y = np.array([r["returns"] for r in data])
    x = np.array([r["x"] for r in data])

    def fit(ids, level):
        totals = y[ids].sum(axis=0)
        action = int(totals.argmax())
        best = float(totals[action])
        split = None
        if level:
            for j in range(x.shape[1]):
                order = ids[np.argsort(x[ids, j], kind="stable")]
                cumulative = y[order].cumsum(axis=0)
                for n in range(minimum, len(ids) - minimum + 1):
                    if x[order[n - 1], j] == x[order[n], j]:
                        continue
                    value = cumulative[n - 1].max() + (totals - cumulative[n - 1]).max() - 0.45
                    if value > best:
                        best = value
                        split = (
                            j,
                            float((x[order[n - 1], j] + x[order[n], j]) / 2),
                            order[:n],
                            order[n:],
                        )
        if split is None:
            return dict(action=action)
        j, t, left, right = split
        return dict(feature=j, threshold=t, left=fit(left, level - 1), right=fit(right, level - 1))

    return fit(np.arange(len(data)), depth)


def ridge(data, penalty):
    heads = []
    for actions in ((0,), (1,), (2, 3)):
        x = np.array([design(r["x"], a) for r in data for a in actions])
        y = np.array([r["returns"][a] for r in data for a in actions])
        reg = np.eye(x.shape[1]) * penalty
        reg[0, 0] = 0.0001
        heads.append(np.linalg.solve(x.T @ x + reg, x.T @ y).tolist())
    return dict(heads=heads)


def fit(folder):
    data = rows(folder)
    use = [r for r in data if r["x"] is not None]
    base = read(OLD / "calibrated.json")
    seeds = sorted({r["seed"] for r in use})
    # Hold entire kickoff seeds out of each fold. Compare two different
    # representations and let subsequent full matches choose between them.
    candidates = []
    report = []
    for family, settings, learner in [
        ("tree", [(1, 7), (2, 7), (3, 5), (3, 3)], lambda rs, s: tree_fit(rs, *s)),
        ("shared ridge", [1, 4, 12, 30], ridge),
    ]:
        trials = []
        for setting in settings:
            actions = []
            goals = passes = 0
            for i in range(5):
                test = set(seeds[i::5])
                model = learner([r for r in use if r["seed"] not in test], setting)
                for r in use:
                    if r["seed"] not in test:
                        continue
                    a = decision(model, r["x"])
                    actions.append(a)
                    goals += r["returns"][a]
                    passes += r["passed"][a]
            trials.append(
                dict(
                    setting=setting, goals=goals, pass_goals=passes, actions=dict(Counter(actions))
                )
            )
        best = max(trials, key=lambda t: (t["goals"], t["pass_goals"]))
        m = learner(use, best["setting"])
        candidates.append(base | dict(distribution=m))
        report.append(dict(family=family, folds=trials, selected=best["setting"]))
    save(DEST / "returns.json", data)
    save(DEST / "fit.json", report)
    save(DEST / "candidates.json", candidates)
    archive(folder, "training")


def freeze(folder):
    if (DEST / "freeze.json").exists():
        raise ValueError("Checkpoint already frozen")
    cfg = read(folder / "launch.json")
    stats = []
    for j in range(1, len(cfg["models"])):
        clean = goals = passes = diff = appearances = 0
        for i, (seed, b, y) in enumerate(cfg["schedule"]):
            if j not in (b, y):
                continue
            g = read(folder / f"game-{i:03}.json")
            for team, idx in (("blue", b), ("yellow", y)):
                if idx != j:
                    continue
                chain = [
                    r for r in g["recoveries"] if r["kind"] == "takeaway" and r["team"] == team
                ]
                appearances += 1
                clean += len(chain)
                goals += sum("goal" in r for r in chain)
                passes += sum(bool(r.get("passes")) for r in chain)
                if b != y:
                    other = "yellow" if team == "blue" else "blue"
                    diff += g["score"].get(team, 0) - g["score"].get(other, 0)
        stats.append(
            dict(
                model=j,
                clean=clean,
                goals=goals,
                pass_goals=passes,
                differential=diff,
                appearances=appearances,
                value=(goals + 0.45 * diff) / appearances,
            )
        )
    best = max(stats, key=lambda s: (s["pass_goals"] > 0, s["value"]))
    save(DEST / "development.json", dict(stats=stats, selected=best["model"]))
    save(DEST / "policy.json", cfg["models"][best["model"]])
    archive(folder, "league")
    files = [
        WORLD,
        CTRL,
        OLD / "calibration-freeze.json",
        OLD / "freeze.json",
        OLD / "calibrated.json",
        DEST / "policy.json",
        DEST / "protocol.json",
        DEST / "returns.json",
        DEST / "development.json",
        ROOT / "scripts/train_counterpass.py",
        ROOT / "scripts/fit_counterpass.py",
        ROOT / "src/roboturk_core/roboturk/counterpass.py",
        ROOT / "src/roboturk_core/roboturk/distribution_value.py",
    ]
    save(DEST / "freeze.json", {str(f.relative_to(ROOT)): sha(f) for f in files})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["fit", "freeze"])
    parser.add_argument("folder", type=Path)
    a = parser.parse_args()
    (fit if a.mode == "fit" else freeze)(a.folder)
