"""Reconstruct contests, sole possession and outlet deliveries from native geometry."""

import numpy as np

TEAMS = ("blue", "yellow")


def measures(states, game):
    xy = states[:, 4:].reshape(-1, 6, 3)
    delta = states[:, None, :2] - xy[:, :, :2]
    c, s = np.cos(xy[:, :, 2]), np.sin(xy[:, :, 2])
    dx, dy = delta[:, :, 0], delta[:, :, 1]
    mouths = (abs(dx * c + dy * s - 0.115) < 0.035) & (abs(-dx * s + dy * c) < 0.045)
    ties = mouths[:, :3].any(1) & mouths[:, 3:].any(1)
    only = np.where(mouths.sum(1) == 1, mouths.argmax(1), -1)
    restarts = {e["tick"] * 4 + 4 for e in game["events"] if e["event"] not in ("start", "kick")}
    kicks = [e for e in game["events"] if e["event"] == "kick"]
    passes = {p["tick"]: p for p in game["passes"]}
    contests = []
    active = None
    tie_run = sole_run = loose_run = 0
    for i, tie in enumerate(ties):
        if i in restarts:
            active = None
            tie_run = sole_run = loose_run = 0
        if tie:
            tie_run += 1
            if tie_run == 20 and active is None:
                active = dict(start=i - 19, last=i, contestants=np.flatnonzero(mouths[i]).tolist())
                contests.append(active)
            if active is not None:
                active["last"] = i
            sole_run = loose_run = 0
        else:
            tie_run = 0
            if active is None:
                continue
            sole_run = (
                sole_run + 1 if only[i] >= 0 and i and only[i - 1] == only[i] else int(only[i] >= 0)
            )
            loose_run = loose_run + 1 if not mouths[i].any() else 0
            if sole_run >= 20:
                actor = int(only[i])
                if actor in active["contestants"] and i - active["last"] <= 300:
                    active.update(winner=actor, recovery=i)
                active = None
            elif loose_run >= 100 or i - active["last"] > 300:
                active = None
    for e in contests:
        if "winner" not in e:
            continue
        actor = e["winner"]
        team = TEAMS[actor // 3]
        start = e["recovery"]
        kick = next((k for k in kicks if k["tick"] * 4 >= start), None)
        if not kick or kick["tick"] * 4 - start > 800:
            continue
        p = passes.get(kick["tick"])
        if not p or p["result"] != "complete" or p["team"] != team or p["passer"] != actor % 3:
            continue
        end = (p["end"] + 1) * 4
        if p["end"] - p["tick"] > 100 or any(p["tick"] < k["tick"] <= p["end"] for k in kicks):
            continue
        if any(start <= t < end for t in restarts):
            continue
        rivals = mouths[start:end, 3:] if team == "blue" else mouths[start:end, :3]
        if rivals.any():
            continue
        receiver = 3 * (actor // 3) + p["receiver"]
        a = p["tick"] * 4
        valid = (only[a:end] == receiver) & (
            np.linalg.norm(states[a:end, :2] - p["ball"], axis=1) >= 0.55
        )
        if (
            len(valid) < 20
            or not (np.convolve(valid.astype(int), np.ones(20, int), "valid") == 20).any()
        ):
            raise ValueError("Reported receiver did not control the physical delivery")
        e.update(pass_tick=p["tick"], received=p["end"], receiver=receiver)
    return dict(
        contested_seconds=float(ties.sum()) * 0.01,
        contests=contests,
        recoveries=sum("winner" in e for e in contests),
        chains=sum("received" in e for e in contests),
        late=sum(
            p["result"] == "complete" and p["end"] * 0.04 >= game["seconds"] - 30
            for p in game["passes"]
        ),
        passes=len(game["passes"]),
        completions=sum(p["result"] == "complete" for p in game["passes"]),
    )
