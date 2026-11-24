"""Compare native 5v5 takeaways with 2 ms roller and body-contact evidence."""

import argparse
from itertools import combinations, groupby
import json
from pathlib import Path

import numpy as np

from audit_five_roles import load_match


def runs_of_control(rollers, restarts):
    # A bit mask with exactly one bit identifies a sole roller. A shared ball
    # interrupts exclusive possession and can still be part of the same duel.
    for (restart, mask), group in groupby(
        enumerate(zip(restarts, rollers, strict=True)), key=lambda item: item[1]
    ):
        first, _ = next(group)
        last = first
        for last, _ in group:
            pass
        value = int(mask)
        if value and value & (value - 1) == 0:
            yield restart, value.bit_length() - 1, first, last + 1


def takeaways(game, rollers, contacts, crashes):
    dt = game["physics_dt"]
    step = round(game["dt"] / dt)
    frames = game["frames"]
    if step * len(frames) != len(rollers) or len(rollers) != len(contacts):
        raise ValueError("Every control tick must contain its native physics samples")
    restarts = np.repeat([f["restart"] for f in frames], step)
    stopped_ticks = [e["tick"] for e in game["events"] if e["event"] in ("goal", "out")]
    # Do not attribute a capture or use the settled positions on a reset tick.
    for tick in stopped_ticks:
        restarts[tick * step : (tick + 1) * step] = -1

    kicks = sorted(
        e["tick"] for e in game["events"] if e["event"] in ("pass", "shot", "restart_pass")
    )
    keys = game["keys"]
    pairs = list(combinations(range(len(keys)), 2))
    taken = []
    previous = None
    for restart, actor, start, stop in runs_of_control(rollers, restarts):
        if restart < 0 or stop - start < round(0.12 / dt):
            continue
        current = restart, actor, start, stop
        if previous is None or restart != previous[0] or keys[previous[1]][0] == keys[actor][0]:
            previous = current
            continue
        old_restart, carrier, old_start, old_stop = previous
        previous = current
        confirmed = start + round(0.24 / dt) - 1
        if confirmed >= stop or start - old_stop > round(1.6 / dt):
            continue
        zone_start = max(old_start, old_stop - round(0.12 / dt))
        first_reach = np.flatnonzero(rollers[zone_start : start + 1] & (1 << actor))
        if not len(first_reach):
            continue
        reach = zone_start + int(first_reach[0])
        carrier_touch = np.flatnonzero(
            rollers[max(old_start, reach - round(0.12 / dt)) : reach + 1] & (1 << carrier)
        )
        if not len(carrier_touch):
            continue
        lead = max(old_start, reach - round(0.32 / dt))
        if any(lead // step <= t <= confirmed // step for t in kicks):
            continue
        if np.any(restarts[lead : confirmed + 1] != old_restart):
            continue
        participants = sum(
            1 << i for i, pair in enumerate(pairs) if actor in pair or carrier in pair
        )
        window = frames[lead // step : confirmed // step + 1]
        body = bool(np.any(contacts[lead : confirmed + 1] & np.uint64(participants)))
        crash = any(lead // step <= e["tick"] <= confirmed // step for e in crashes)
        taken.append(
            dict(
                actor=keys[actor],
                carrier=keys[carrier],
                presser=any(frame["roles"][actor] == "Presser" for frame in window),
                trained_tackle=any([actor, carrier] in frame.get("tackle", []) for frame in window),
                contact=body,
                ref_crash=crash,
                clean=not (body or crash),
                reach_s=round((reach + 1) * dt, 3),
                confirmed_s=round((confirmed + 1) * dt, 3),
            )
        )
    return taken


def summarize(folder, game):
    ref = json.loads((folder / "official/result.json").read_text())
    if ref["frames"] != len(game["frames"]) or ref["seed"] != game["seed"]:
        raise ValueError(f"AutoRef did not adjudicate the same full match in {folder}")
    with np.load(folder / "possession.npz") as samples:
        events = takeaways(game, samples["rollers"], samples["contacts"], ref["crashes"])
    return dict(
        goals=sum(game["score"]),
        score=game["score"],
        ref_crash_calls=len(ref["crashes"]),
        body_contact_episodes=game["robot_clearance"]["contact_episodes"],
        min_gap_m=game["robot_clearance"]["min_gap_m"],
        clean_takeaways=sum(e["clean"] for e in events),
        clean_presser_takeaways=sum(e["clean"] and e["presser"] for e in events),
        clean_trained_takeaways=sum(e["clean"] and e["trained_tackle"] for e in events),
        direct_transfers=len(events),
        tackle_ticks=sum(bool(f.get("tackle")) for f in game["frames"]),
        transfers=events,
        crashes=ref["crashes"],
    )


def compare(baseline, trained):
    current_paths = {p.name: p for p in baseline.iterdir() if p.is_dir()}
    fitted_paths = {p.name: p for p in trained.iterdir() if p.is_dir()}
    if not current_paths or current_paths.keys() != fitted_paths.keys():
        raise ValueError("Use the same nonempty set of kickoff directories")
    pairs = []
    for name in sorted(current_paths):
        a, b = current_paths[name], fitted_paths[name]
        g, x = load_match(a)
        h, y = load_match(b)
        if (
            any(g[k] != h[k] for k in ("seed", "seconds", "dt", "physics_dt", "keys", "sign"))
            or not np.array_equal(x[0], y[0])
            or g["frames"][0]["opening_velocities"] != h["frames"][0]["opening_velocities"]
        ):
            raise ValueError(f"Kickoff state or clock differs for {name}")
        pairs.append(
            dict(
                seed=g["seed"],
                seconds=g["seconds"],
                current=summarize(a, g),
                trained=summarize(b, h),
            )
        )
    if len({pair["seed"] for pair in pairs}) != len(pairs):
        raise ValueError("Each kickoff must have its own seed")
    keys = (
        "goals",
        "ref_crash_calls",
        "body_contact_episodes",
        "clean_takeaways",
        "clean_presser_takeaways",
        "clean_trained_takeaways",
        "direct_transfers",
        "tackle_ticks",
    )
    totals = {}
    for arm in ("current", "trained"):
        totals[arm] = {k: sum(p[arm][k] for p in pairs) for k in keys}
        totals[arm]["min_gap_m"] = min(p[arm]["min_gap_m"] for p in pairs)
    return dict(pairs=pairs, totals=totals, seconds_per_arm=sum(p["seconds"] for p in pairs))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("current", type=Path)
    parser.add_argument("trained", type=Path)
    a = parser.parse_args()
    print(json.dumps(compare(a.current, a.trained), indent=2))
