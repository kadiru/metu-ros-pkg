"""Rebuild the selected opening estimates and verify the physical trial counts."""

import gzip
import json
from roboturk.attack_opening import fit, training_example
from run_shield_attack import PLAN, R, ROOT, V1, digest, read, verify
from select_recovery_opening import compare


def main():
    verify()
    for name, sha in read(R / "freeze.json").items():
        assert digest(ROOT / name) == sha, name
    plan = read(PLAN)
    payload = json.loads(gzip.decompress((R / "training.json.gz").read_bytes()))
    model = read(V1 / "policy.json")
    controls = plan["controls"]
    stats = [[0, 0, 0] for _ in controls]
    games = payload["motor_games"]
    assert len(games) == len(controls) * plan["motors_layouts"]
    for i, g in enumerate(games):
        idx = (i + i // len(controls)) % len(controls)
        assert g["control"] == idx and g["seed"] == plan["motors_first"] + i // len(controls)
        if g["conversions"] and "goal" in (e := g["conversions"][0]):
            for j, count in enumerate([1, bool(e["passes"]), e["dribble_m"] >= 0.18]):
                stats[idx][j] += int(count)
    best = max(range(len(controls)), key=lambda i: stats[i])
    assert stats == payload["motor_returns"] and best == payload["best_motor"]
    model["control"] = controls[best]
    rows = payload["branches"]
    assert len(rows) == 3 * plan["branches_layouts"]
    examples = []
    for i, g in enumerate(rows):
        assert g["seed"] == plan["branches_first"] + i // 3
        ex = training_example(g)
        assert ex == g["opening_example"]
        if ex:
            assert ex["action"] == i % 3
            examples.append(ex)
    scores = [
        compare(rows, n, p, b) for n in (8, 18, 36) for p in (4, 10, 20) for b in (0, 3, 6, 9)
    ]
    assert scores == payload["candidates"]
    diverse = [r for r in scores if min(r["assisted"], r["carried"]) >= 0.08 * r["matched"]]
    selected = max(diverse or scores, key=lambda r: r["mean_return"])
    assert selected == payload["opening_selected"]
    d = payload["development"]
    choices = [k for k, v in d.items() if min(v["totals"]["assisted"], v["totals"]["carried"]) > 0]
    winner = max(choices or list(d), key=lambda k: d[k]["totals"]["first"])
    assert winner == payload["selected"]
    if winner == "opening":
        model["opening"] = fit(examples, selected["neighbors"])
        model["opening"].update(
            pass_neighbors=selected["pass_neighbors"],
            pass_success_bonus=selected["pass_success_bonus"],
        )
    assert model == read(R / "policy.json")
    print(len(games), len(rows), stats, winner)


if __name__ == "__main__":
    main()
