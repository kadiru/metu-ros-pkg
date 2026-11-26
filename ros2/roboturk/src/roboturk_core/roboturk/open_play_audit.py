"""Attribute open-play goals to physical changes of possession at 100 Hz."""

import numpy as np

from .contest_metrics import measures


def recoveries(states, game):
    robots = states[:, 4:].reshape(-1, 6, 3)
    delta = states[:, None, :2] - robots[:, :, :2]
    cosine, sine = np.cos(robots[:, :, 2]), np.sin(robots[:, :, 2])
    x, y = delta[:, :, 0], delta[:, :, 1]
    mouths = (abs(x * cosine + y * sine - 0.115) < 0.035) & (abs(y * cosine - x * sine) < 0.045)
    solo = np.where(mouths.sum(axis=1) == 1, mouths.argmax(axis=1), -1)
    stops = {e["tick"] * 4 + 3: e for e in game["events"] if e["event"] not in ("start", "kick")}
    contests = measures(states, game)["contests"]
    rows = {
        c["recovery"]: dict(sample=c["recovery"], winner=c["winner"], kind="contest")
        for c in contests
        if "recovery" in c
    }
    side = previous = None
    age = 0
    for i, actor in enumerate(solo):
        if i - 1 in stops:
            previous = side = None
            age = 0
        age = age + 1 if actor == previous else 1
        previous = actor
        if actor < 0 or age != 20:
            continue
        next_side = int(actor) // 3
        if side is not None and side != next_side:
            rows.setdefault(i, dict(sample=i, winner=int(actor), kind="takeaway"))
        side = next_side
    kicks = [e for e in game["events"] if e["event"] == "kick"]
    passes = {p["tick"]: p for p in game["passes"]}
    ordered = sorted(rows)
    for index, start in enumerate(ordered):
        row = rows[start]
        side = row["winner"] // 3
        team = ("blue", "yellow")[side]
        row.update(team=team, outcome="match ended")
        end = next((s for s in stops if s >= start), len(states) - 1)
        rival = mouths[start : end + 1, 3 * (1 - side) : 3 * (1 - side) + 3].any(axis=1)
        if rival.any():
            row.update(outcome="opponent touch", end=start + int(np.flatnonzero(rival)[0]))
            continue
        row["end"] = end
        event = stops.get(end, {}).get("event")
        if event != "goal":
            row["outcome"] = event or "match ended"
            continue
        direction = (1 if game["seed"] % 2 == 0 else -1) * (1 if side == 0 else -1)
        sequence = [k for k in kicks if start <= k["tick"] * 4 <= end]
        if (
            not sequence
            or sequence[-1]["team"] != team
            or sequence[-1]["speed"] != 4
            or direction * states[end, 0] <= 4.5215
            or abs(states[end, 1]) >= 0.4785
            or end - sequence[-1]["tick"] * 4 > 300
        ):
            row["outcome"] = "unattributed goal"
            continue
        actor = row["winner"] % 3
        completed = []
        for kick in sequence[:-1]:
            p = passes.get(kick["tick"])
            if (
                not p
                or kick["speed"] != 2.2
                or kick["robot"] != actor
                or kick["team"] != team
                or p["result"] != "complete"
                or p["team"] != team
            ):
                break
            # measures() checks contest passes only. Verify every link here.
            window = slice(kick["tick"] * 4, (p["end"] + 1) * 4)
            held = (solo[window] == 3 * side + p["receiver"]) & (
                np.linalg.norm(states[window, :2] - p["ball"], axis=1) >= 0.55
            )
            if len(held) < 20 or np.convolve(held.astype(int), np.ones(20), "valid").max() < 20:
                break
            completed.append(kick["tick"])
            actor = p["receiver"]
        else:
            if sequence[-1]["robot"] == actor and (
                index + 1 == len(ordered) or ordered[index + 1] > end
            ):
                row.update(outcome="goal", goal=end, scorer=actor, passes=completed)
                continue
        row["outcome"] = "unverified scoring chain"
    return [rows[s] for s in ordered]
