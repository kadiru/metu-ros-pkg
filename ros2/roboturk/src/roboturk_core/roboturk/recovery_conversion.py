"""Independent physical attribution of goals to an uninterrupted recovery."""

import numpy as np
from .contest_metrics import measures


def conversions(states, game):
    previous = measures(states, game)
    robots = states[:, 4:].reshape(-1, 6, 3)
    d = states[:, None, :2] - robots[:, :, :2]
    c, s = np.cos(robots[:, :, 2]), np.sin(robots[:, :, 2])
    held = (abs(c * d[:, :, 0] + s * d[:, :, 1] - 0.115) < 0.035) & (
        abs(-s * d[:, :, 0] + c * d[:, :, 1]) < 0.045
    )
    kicks = [e for e in game["events"] if e["event"] == "kick"]
    stops = [e for e in game["events"] if e["event"] not in ("kick", "start")]
    deliveries = {p["tick"]: p for p in game["passes"]}
    used = set()
    rows = []
    sign = 1 if game["seed"] % 2 == 0 else -1
    for contest in previous["contests"]:
        row = dict(start=contest["start"])
        rows.append(row)
        if "recovery" not in contest:
            continue
        start, actor = contest["recovery"], contest["winner"]
        team = "blue" if actor < 3 else "yellow"
        row.update(recovery=start, winner=actor, team=team)
        stop = next((e for e in stops if e["tick"] * 4 + 3 >= start), None)
        if not stop or stop["event"] != "goal" or stop["tick"] in used:
            continue
        end = stop["tick"] * 4 + 3
        direction = sign if team == "blue" else -sign
        rival = held[start : end + 1, 3:] if team == "blue" else held[start : end + 1, :3]
        if (
            end - start > 1800
            or rival.any()
            or direction * states[end, 0] <= 4.5215
            or abs(states[end, 1]) >= 0.4785
        ):
            continue
        sequence = [e for e in kicks if start <= 4 * e["tick"] <= end]
        if not sequence or sequence[-1]["speed"] != 4 or (end - sequence[-1]["tick"] * 4) > 300:
            continue
        if any(e["team"] != team for e in sequence):
            continue
        carrier = actor % 3
        passes = []
        for k in sequence[:-1]:
            p = deliveries.get(k["tick"])
            if k["robot"] != carrier or k["speed"] != 2.2 or not p or p["result"] != "complete":
                break
            idx = (0 if team == "blue" else 3) + p["receiver"]
            segment = slice(k["tick"] * 4, (p["end"] + 1) * 4)
            valid = (
                held[segment, idx]
                & (held[segment].sum(1) == 1)
                & (np.linalg.norm(states[segment, :2] - p["ball"], axis=1) >= 0.55)
            )
            if len(valid) < 20 or np.convolve(valid.astype(int), np.ones(20), "valid").max() < 20:
                break
            carrier = p["receiver"]
            passes.append(p["tick"])
        else:
            if sequence[-1]["robot"] != carrier:
                continue
            opts = [
                v
                for v in game.get("options", [])
                if v["team"] == team and start <= v["tick"] * 4 <= sequence[-1]["tick"] * 4
            ]
            dribble = 0.0
            for v in opts:
                if v["action"] != "dribble":
                    continue
                a = v["tick"] * 4
                b = min(v["until"] * 4, end + 1)
                key = (0 if team == "blue" else 3) + v["actor"]
                ok = held[a:b, key] & (held[a:b].sum(1) == 1)
                bad = np.flatnonzero(~ok)
                n = int(bad[0]) if len(bad) else len(ok)
                if n > 1:
                    ball_travel = np.linalg.norm(states[a : a + n, :2] - states[a, :2], axis=1)
                    robot_travel = np.linalg.norm(
                        robots[a : a + n, key, :2] - robots[a, key, :2], axis=1
                    )
                    dribble = max(dribble, float(np.minimum(ball_travel, robot_travel).max()))
            row.update(
                goal=end,
                shot=sequence[-1]["tick"],
                scorer=carrier,
                passes=passes,
                dribble_m=round(dribble, 3),
                options=[v["action"] for v in opts],
            )
            used.add(stop["tick"])
    return rows
