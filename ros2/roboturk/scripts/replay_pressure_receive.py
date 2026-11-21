"""Record and verify the original first-touch loss and the trained team's response."""

import argparse
from pathlib import Path
import numpy as np

from train_pressure_receive import PRIOR, WEBOTS, WORLD, ROOT, read, save, sha, run_matches


def first_touch(states, game):
    row = game["opening"]
    begin, end = row["kick"] * 4, (row["end"] + 1) * 4
    segment = states[begin:end]
    robots = segment[:, 4:].reshape(-1, 6, 3)
    offset = 3 * int(row["chain"]["team"] == "yellow")
    runner = offset + row["receiver"]
    d = segment[:, None, :2] - robots[:, :, :2]
    c, s = np.cos(robots[:, :, 2]), np.sin(robots[:, :, 2])
    x, y = d[:, :, 0], d[:, :, 1]
    held = (abs(x * c + y * s - 0.115) < 0.035) & (abs(y * c - x * s) < 0.045)
    rivals = np.arange(3) + 3 - offset
    j, slot = np.argwhere(held[:, rivals])[0]
    opponent = rivals[slot]
    sole = held[:j, runner] & (held[:j].sum(1) == 1)
    n = int(j - (np.flatnonzero(~sole)[-1] + 1)) if (~sole).any() else int(j)
    gap = np.linalg.norm(robots[:, runner, :2] - robots[:, opponent, :2], axis=1)
    return dict(
        seed=game["seed"],
        receiver=int(runner % 3),
        team=row["chain"]["team"],
        opponent=int(opponent % 3),
        exclusive_samples=n,
        first_touch_sample=int(begin + j - n),
        interception_sample=int(begin + j),
        opponent_gap_first_m=float(gap[j - n]),
        opponent_gap_loss_m=float(gap[j]),
        delay_from_release_s=(int(j) + 1) * 0.01,
        receiver_still_holds=bool(held[j, runner]),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("side", choices=["previous", "trained"])
    parser.add_argument("pair", type=Path)
    parser.add_argument("--historical", type=Path)
    parser.add_argument("--webots", type=Path, default=WEBOTS)
    a = parser.parse_args()
    old = PRIOR / "qualification-1"
    src = old if a.side == "previous" else a.historical
    slot = 14 if a.side == "previous" else 0
    world = WORLD.with_name("ssl_lead.wbt") if a.side == "previous" else WORLD
    d = (a.pair / a.side).resolve()
    d.mkdir(parents=True, exist_ok=False)
    cfg = read(src / "launch.json")
    cfg.update(root=str(ROOT), record=True, cases=1, schedule=[cfg["schedule"][slot]])
    assert cfg["schedule"][0][0] == 13300014 and cfg["webots_sha256"] == sha(a.webots)
    save(d / "launch.json", cfg)
    run_matches(d, cfg, a.webots, world, True)
    states = np.load(d / "trace-000.npz")["states"]
    original = np.load(src / f"trace-{slot:03}.npz")["states"]
    repeat = read(d / "game-000.json")
    expected = read(src / f"game-{slot:03}.json")
    assert np.array_equal(states, original)
    assert all(
        repeat[k] == expected[k] for k in ["events", "options", "passes", "opening", "plans"]
    )
    proof = dict(samples=len(states), max_state_delta=0, events_plans_options_match=True)
    if a.side == "previous":
        save(a.pair / "first-touch.json", first_touch(states, repeat) | dict(reproduction=proof))
    else:
        loss = read(a.pair / "first-touch.json")
        before = dict(
            outcome="First touch intercepted", elapsed_s=(1 + loss["interception_sample"]) * 0.01
        )
        row = repeat["opening"]
        save(
            a.pair / "replay.json",
            dict(
                stem="matched-reception",
                before=before,
                play=row["chain"],
                original=loss,
                trained=row,
                reproduction=proof,
            ),
        )


if __name__ == "__main__":
    main()
