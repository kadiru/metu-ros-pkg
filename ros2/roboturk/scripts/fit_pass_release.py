"""Learn release-time decisions from matched native actions and full-match self-play."""

import argparse
from collections import Counter
from pathlib import Path

import numpy as np
from roboturk.pass_release import choose_release
from fit_counterpass import archive, tree_fit
from roboturk.takeaway import select
from train_pass_release import CTRL, DEST, ROOT, WORLD, all_games, attempts, read, save, sha


def result(game, ref, duration):
    chain = next(
        r
        for r in game["recoveries"]
        if r["kind"] == "takeaway" and r["sample"] == ref["sample"] and r["team"] == ref["team"]
    )
    until = min(chain["end"], duration * 100 - 1)
    passes = [
        p
        for p in game["passes"]
        if p["team"] == ref["team"] and ref["tick"] <= p["tick"] <= until / 4
    ]
    return dict(
        goal=int(chain.get("goal", duration * 100) < duration * 100),
        complete=int(any(p["result"] == "complete" and 4 * p["end"] <= until for p in passes)),
        intercepted=int(
            any(p["result"] == "intercepted" and 4 * p["end"] < duration * 100 for p in passes)
        ),
        attempted=len(passes),
        outcome=chain["outcome"] if chain["end"] < duration * 100 else "match ended",
    )


def training_rows(folder):
    cfg = read(folder / "launch.json")
    old = ROOT / read(DEST / "protocol.json")["archive"]
    rows = []
    for i in range(0, cfg["cases"], 3):
        ref = cfg["references"][i]
        results = [result(read(old / ref["source"]), ref, cfg["durations"][i])]
        obs = None
        for j in range(3):
            g = read(folder / f"game-{i + j:03}.json")
            (review,) = g["release_decisions"]
            assert (
                cfg["references"][i + j] == ref
                and review["tick"] == ref["tick"]
                and review["action"] == j + 1
            )
            assert obs is None or obs == review["x"]
            obs = review["x"]
            results.append(result(g, ref, cfg["durations"][i]))
        rows.append(dict(reference=ref, x=obs, results=results))
    return rows


def neighborhood(data, neighbors, bandwidth, prior):
    x = np.asarray([r["x"] for r in data])
    floor = [
        0.25,
        0.09,
        0.16,
        0.30,
        0.30,
        0.12,
        0.035,
        0.25,
        0.25,
        0.18,
        0.20,
        0.30,
        1.0,
        0.6,
        0.22,
        0.35,
        0.2,
        0.5,
        0.55,
    ]
    scale = np.maximum(np.percentile(x, 75, axis=0) - np.percentile(x, 25, axis=0), floor)
    y = np.asarray([r["returns"] for r in data])
    return dict(
        examples=x.tolist(),
        scale=scale.tolist(),
        advantages=(y - y[:, :1]).tolist(),
        neighbors=neighbors,
        bandwidth=bandwidth,
        release_prior=prior,
    )


def fit(folder, out):
    rows = training_rows(folder)
    archive_rows = read(DEST / "interceptions.json")

    def stratum(s):
        return s if s in ("complete", "intercepted") else "other"

    originals = Counter(stratum(r["outcome"]) for r in archive_rows)
    sampled = Counter(stratum(r["reference"]["outcome"]) for r in rows)
    seeds = sorted({r["reference"]["seed"] for r in rows})
    policies = []
    report = []
    for name, complete, intercept, change in [
        ("balanced", 0.60, 0.80, 0.12),
        ("preserve passing", 1.2, 0.70, 0.18),
        ("finish", 0.32, 0.90, 0.08),
    ]:
        data = []
        for row in rows:
            group = stratum(row["reference"]["outcome"])
            weight = originals[group] * len(rows) / (sampled[group] * len(archive_rows))
            scores = [
                weight
                * (
                    s["goal"]
                    + complete * s["complete"]
                    - intercept * s["intercepted"]
                    - change * (j != 0)
                )
                for j, s in enumerate(row["results"])
            ]
            data.append(dict(seed=row["reference"]["seed"], x=row["x"], returns=scores))
        trials = []
        for depth, leaf in [(1, 5), (2, 4), (3, 3), (4, 3), (3, 5), (4, 4)]:
            value = 0
            counts = Counter()
            for fold in range(5):
                held = set(seeds[fold::5])
                model = tree_fit([r for r in data if r["seed"] not in held], depth, leaf)
                for r in data:
                    if r["seed"] in held:
                        a = select(model, r["x"])
                        value += r["returns"][a]
                        counts[a] += 1
            trials.append(dict(depth=depth, leaf=leaf, value=value, choices=dict(counts)))
        best = max(trials, key=lambda t: t["value"])
        model = tree_fit(data, best["depth"], best["leaf"])
        report.append(dict(name=name, trials=trials, selected=best))
        policies.append(
            read(ROOT / read(DEST / "protocol.json")["baseline"]) | dict(pass_release=model)
        )
    # Local action advantages preserve useful exceptions that a single split can group away.
    data = [
        dict(
            seed=r["reference"]["seed"],
            x=r["x"],
            returns=[
                q["goal"] + 0.8 * q["complete"] - 0.85 * q["intercepted"] - 0.10 * (j != 0)
                for j, q in enumerate(r["results"])
            ],
        )
        for r in rows
    ]
    trials = []
    for k, bw, prior in [(3, 0.30, 0.08), (5, 0.6, 0.14), (7, 0.8, 0.25), (10, 1.0, 0.30)]:
        value = 0
        choices = Counter()
        for fold in range(5):
            held = set(seeds[fold::5])
            model = neighborhood([r for r in data if r["seed"] not in held], k, bw, prior)
            for r in data:
                if r["seed"] in held:
                    action = choose_release(model, r["x"])
                    value += r["returns"][action]
                    choices[action] += 1
        trials.append(dict(k=k, bw=bw, prior=prior, value=value, choices=dict(choices)))
    best = max(trials, key=lambda t: t["value"])
    model = neighborhood(data, best["k"], best["bw"], best["prior"])
    report.append(dict(name="local advantages", trials=trials, selected=best))
    policies.append(
        read(ROOT / read(DEST / "protocol.json")["baseline"]) | dict(pass_release=model)
    )
    save(out / "candidates.json", policies)
    save(out / "returns.json", rows)
    save(out / "fit.json", report)
    archive(folder, "training", out)


def league_stats(games, model):
    score = Counter()
    for g in games:
        for team in ("blue", "yellow"):
            if g[team + "_model"] != model:
                continue
            other = "blue" if team == "yellow" else "yellow"
            score["sides"] += 1
            if g[other + "_model"] != model:
                score["head_to_head_margin"] += g["score"].get(team, 0) - g["score"].get(other, 0)
            for r in g["recoveries"]:
                if r["team"] == team and r["kind"] == "takeaway":
                    score["goals"] += int("goal" in r)
            for r, p in attempts(g):
                if p["team"] == team:
                    score["attempts"] += 1
                    score["complete"] += int(p["result"] == "complete" and 4 * p["end"] <= r["end"])
                    score["intercepted"] += int(p["result"] == "intercepted")
            for d in g["release_decisions"]:
                if d["team"] == team:
                    score["action_" + str(d["action"])] += 1
    return dict(score)


def freeze(folder, out):
    cfg = read(folder / "launch.json")
    assert cfg["models"][1:] == read(out / "candidates.json")
    games = [g for g, _ in all_games(folder)]
    assert len(games) == cfg["cases"] and (folder / "summary.json").exists()
    stats = [league_stats(games, i) for i in range(1, len(cfg["models"]))]
    reference = folder.parent / "dev-control"
    controls = [g for g, _ in all_games(reference)]
    if len(controls) != 6 or not (reference / "summary.json").exists():
        raise ValueError("Run dev-control for the six matched development kickoffs")
    assert {g["seed"] for g in controls} == {g["seed"] for g in games}

    def mirror_score(group):
        clean = [r for g in group for r in g["recoveries"] if r["kind"] == "takeaway"]
        passes = [(r, p) for g in group for r, p in attempts(g)]
        return dict(
            goals=sum("goal" in r for r in clean),
            attempts=len(passes),
            completed=sum(
                p["result"] == "complete" and 4 * p["end"] <= r["end"] for r, p in passes
            ),
            interceptions=sum(p["result"] == "intercepted" for _, p in passes),
        )

    baseline = mirror_score(controls)
    for i, s in enumerate(stats, 1):
        m = mirror_score([g for g in games if g["blue_model"] == g["yellow_model"] == i])
        s["mirror"] = m
        s["development_checks"] = dict(
            rate=m["attempts"] > 0
            and m["interceptions"] * baseline["attempts"]
            < baseline["interceptions"] * m["attempts"],
            keep_completions=2 * m["completed"] >= baseline["completed"],
            goals=m["goals"] >= baseline["goals"],
        )
    for s in stats:
        s["value"] = (
            s["goals"]
            + 0.6 * s.get("complete", 0)
            - s.get("intercepted", 0)
            + 0.45 * s["head_to_head_margin"]
        ) / s["sides"]
    # Selection uses only separate development kickoffs.
    winner = max(
        range(len(stats)),
        key=lambda i: (
            all(stats[i]["development_checks"].values()),
            stats[i].get("complete", 0) > 0,
            stats[i]["value"],
        ),
    )
    save(
        out / "development.json", dict(models=stats, baseline_mirrors=baseline, selected=winner + 1)
    )
    save(out / "policy.json", cfg["models"][winner + 1])
    archive(folder, "development-games", out)
    archive(reference, "development-control", out)
    files = [
        CTRL,
        WORLD,
        DEST / "protocol.json",
        DEST / "interceptions.json",
        out / "policy.json",
        out / "returns.json",
        out / "development.json",
        ROOT / "policies/counterpass_v1/freeze.json",
        ROOT / "src/roboturk_core/roboturk/pass_release.py",
        *(
            ROOT / "scripts" / name
            for name in (
                "train_pass_release.py",
                "fit_pass_release.py",
                "audit_pass_release.py",
                "takeaway_native.py",
                "compare_takeaways.py",
                "fit_counterpass.py",
            )
        ),
    ]
    save(out / "freeze.json", {str(f.relative_to(ROOT)): sha(f) for f in files})


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=["fit", "freeze"])
    p.add_argument("folder", type=Path)
    p.add_argument("--output", type=Path, default=DEST)
    a = p.parse_args()
    out = a.output.resolve()
    out.relative_to(ROOT)
    out.mkdir(exist_ok=True, parents=True)
    if (out / "freeze.json").exists():
        p.error("Use a new output for another training run")
    if not (a.folder / "summary.json").exists():
        p.error("Incomplete native schedule")
    (fit if a.mode == "fit" else freeze)(a.folder, out)
