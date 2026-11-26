"""Check both teams on the same 40 initial states and count losses before release."""

import argparse
from pathlib import Path
from collections import Counter
import numpy as np
from train_recovery_touch import panel, touch_result, read, save


def compare(new, old):
    cfg = read(new / "launch.json")
    prior = read(old / "launch.json")
    assert cfg["schedule"] == prior["schedule"] and cfg["press"] and prior["press"]
    assert cfg["protocol"] == prior["protocol"] and len(cfg["schedule"]) == 40
    rows = [[], []]
    actions = Counter()
    for i, (seed, _, _) in enumerate(cfg["schedule"]):
        traces = [np.load(d / f"trace-{i:03}.npz")["states"] for d in (new, old)]
        assert np.array_equal(traces[0][0], traces[1][0]), seed
        for team, (states, d) in enumerate(zip(traces, (new, old), strict=True)):
            g = read(d / f"game-{i:03}.json")
            assert g["seed"] == seed and len(states) == 1800
            r = touch_result(states, g, cfg["protocol"])
            assert r == g["opening"]
            rows[team].append(r)
            if team == 0 and "winner" in r["chain"]:
                end = r.get("first_pass")
                if end is None:
                    end = next(
                        (e["tick"] for e in g["events"] if e["event"] not in ("kick", "start")), 450
                    )
                actor = r["chain"]["winner"] % 3
                for v in g["first_touches"]:
                    if (
                        v["team"] == r["chain"]["team"]
                        and v["actor"] == actor
                        and v["first"] <= end
                    ):
                        actions[v["action"]] += max(0, min(end, v["last"]) - v["first"] + 1)
    return dict(
        initial_states_match=40,
        trained=panel(rows[0]),
        prior=panel(rows[1]),
        trained_carrier_action_ticks=dict(actions),
        sides=dict(Counter(r["chain"]["team"] for r in rows[0] if r["qualified"])),
        pairs=[
            dict(
                seed=x["seed"],
                old_goal=y["qualified"],
                new_goal=x["qualified"],
                old_loss=y["prepass_loss"],
                new_loss=x["prepass_loss"],
                old_outcome=y["outcome"],
                new_outcome=x["outcome"],
            )
            for x, y in zip(*rows, strict=True)
        ],
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("trained", type=Path)
    p.add_argument("previous", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    result = compare(a.trained, a.previous)
    save(a.output, result)
    print({k: v for k, v in result.items() if k != "pairs"})
