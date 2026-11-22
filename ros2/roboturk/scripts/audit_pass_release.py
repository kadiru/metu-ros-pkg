"""Compare the two frozen teams, including every intercepted release and its rate."""

import argparse
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np
from compare_takeaways import compare, trace, all_games
from fit_pass_release import result
from train_pass_release import attempts, read, save


def counts(g, team=None):
    c = Counter()
    chains = [r for r in g["recoveries"] if r["kind"] == "takeaway" and team in (None, r["team"])]
    c["takeaways"] = len(chains)
    for r in chains:
        c["goals"] += int("goal" in r)
        c["pass_goals"] += int(bool(r.get("passes")))
    for r, p in attempts(g):
        if team not in (None, p["team"]):
            continue
        c["attempts"] += 1
        c["completed"] += int(p["result"] == "complete" and 4 * p["end"] <= r["end"])
        c["intercepted"] += int(p["result"] == "intercepted")
    return c


def latest_passes(g, d, r):
    return [
        p for p in g["passes"] if p["team"] == d["team"] and d["tick"] <= p["tick"] <= r["end"] / 4
    ]


def review_outcomes(g):
    records = []
    for d in g["release_decisions"]:
        r = next(
            r for r in g["recoveries"] if r["sample"] == d["sample"] and r["team"] == d["team"]
        )
        ps = latest_passes(g, d, r)
        p = ps[0] if ps else None
        records.append(
            dict(
                sample=d["sample"],
                tick=d["tick"],
                team=d["team"],
                actor=d["actor"],
                action=d["action"],
                intended=d["receiver"],
                alternative=d["alternate"],
                released_to=p["receiver"] if p else None,
                release_tick=p["tick"] if p else None,
                completed=bool(p and p["result"] == "complete" and 4 * p["end"] <= r["end"]),
                outcome=r["outcome"],
                goal=r.get("goal"),
                end=r["end"],
            )
        )
    return records


def audit(control, trained):
    a = all_games(control)
    b = all_games(trained)
    assert len(a) == 40 and len(b) == 56
    out = compare(control, trained)
    source = {g["seed"]: (g, f) for g, f in a}
    totals = defaultdict(Counter)
    decisions = defaultdict(Counter)
    pairs = {}
    prefixes = []
    demo = []
    outcomes = {}
    for g, f in a + b:
        blue, yellow = g["blue_model"], g["yellow_model"]
        arm = "mixed" if blue != yellow else "trained" if blue else "previous"
        s = trace(f)
        assert len(s) == 9000
        if arm == "mixed":
            for team in ("blue", "yellow"):
                totals["new_vs_old" if g[team + "_model"] else "old_vs_new"].update(counts(g, team))
        else:
            n = counts(g)
            totals[arm].update(n)
            pairs.setdefault(g["seed"], {})[arm] = dict(n)
        if arm == "previous":
            continue
        old, rf = source[g["seed"]]
        prior = trace(rf)
        changed = next((d for d in g["release_decisions"] if d["action"] != 0), None)
        through = 4 * changed["tick"] if changed else len(s)
        assert g["initial_world"] == old["initial_world"] and np.array_equal(
            s[:through], prior[:through]
        )
        prefixes.append(dict(seed=g["seed"], teams=[blue, yellow], identical=through))
        reviews = review_outcomes(g)
        outcomes[f.name] = reviews
        for d in reviews:
            key = (arm if arm != "mixed" else "new_vs_old") + "_" + str(d["action"])
            tally = decisions[key]
            tally["choices"] += 1
            tally["goals"] += int(d["goal"] is not None)
            tally["completed"] += int(d["completed"])
            if d["action"] == 1 and d["release_tick"] is not None:
                assert d["release_tick"] >= d["tick"] + 8 and d["released_to"] == d["intended"]
                tally["delayed_kicks"] += 1
            if (
                d["action"] == 2
                and d["release_tick"] is not None
                and d["released_to"] == d["alternative"]
            ):
                tally["switched_kicks"] += 1
            if arm != "trained" or d["action"] == 0 or d["goal"] is None:
                continue
            match = next(
                (
                    p
                    for _, p in attempts(old)
                    if p["tick"] == d["tick"]
                    and p["team"] == d["team"]
                    and p["result"] == "intercepted"
                ),
                None,
            )
            paired = bool(match and np.array_equal(s[: 4 * d["tick"]], prior[: 4 * d["tick"]]))
            demo.append(
                dict(seed=g["seed"], challenger=f.name, control=rf.name, paired=paired, decision=d)
            )
    assert len(pairs) == 40 and len(prefixes) == 56
    for c in totals.values():
        c["interception_rate"] = c["intercepted"] / c["attempts"] if c["attempts"] else None
    new, old = totals["trained"], totals["previous"]
    wdl = out["head_to_head_record"]
    gates = dict(
        rate=new["attempts"] > 0
        and old["attempts"] > 0
        and new["interception_rate"] < old["interception_rate"],
        completions=2 * new["completed"] >= old["completed"],
        goals=new["goals"] >= old["goals"],
        head_to_head=wdl.get("won", 0) >= wdl.get("lost", 0),
    )
    # Resample whole matched kickoffs, preserving attempt volume within each start.
    raw = np.array(
        [
            [p[a].get(k, 0) for a in ("previous", "trained") for k in ("intercepted", "attempts")]
            for p in pairs.values()
        ]
    )
    sampled = raw[np.random.default_rng(197).integers(0, len(raw), size=(20000, len(raw)))].sum(
        axis=1
    )
    good = (sampled[:, 1] > 0) & (sampled[:, 3] > 0)
    ci = np.percentile(
        sampled[good, 2] / sampled[good, 3] - sampled[good, 0] / sampled[good, 1], [2.5, 97.5]
    ).tolist()
    return out | dict(
        pass_counts=totals,
        decisions=decisions,
        promotion_checks=gates,
        promoted=all(gates.values()),
        pass_rate_difference_95=ci,
        pass_pairs=pairs,
        prefixes=prefixes,
        demo_candidates=demo,
        release_outcomes=outcomes,
    )


def historical(folder):
    cfg = read(folder / "launch.json")
    rows = []
    assert len(cfg["schedule"]) == 43 and (folder / "summary.json").exists()
    for i, (g, f) in enumerate(all_games(folder)):
        ref = cfg["references"][i]
        (d,) = review_outcomes(g)
        rows.append(dict(reference=ref, decision=d, result=result(g, ref, cfg["durations"][i])))
    return rows


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("trained", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--control", type=Path)
    a = p.parse_args()
    save(a.output, audit(a.control, a.trained) if a.control else historical(a.trained))
