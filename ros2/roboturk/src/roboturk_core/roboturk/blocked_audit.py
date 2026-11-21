"""Reconstruct blocked-lane pass goals from Webots samples and physical kick events."""

from .blocked_outlet import geometry
from .recovery_conversion import conversions


def opening_result(states, game, protocol):
    chains = conversions(states, game)
    row = dict(seed=game["seed"], qualified=False)
    if not chains or chains[0]["start"] != 0:
        return row | {"outcome": "no opening contest"}
    chain = chains[0]
    row["chain"] = chain
    if "winner" not in chain:
        return row | {"outcome": "no confirmed recovery"}
    team = chain["team"]
    idx = chain["winner"]
    side = idx // 3
    direction = (1 if game["seed"] % 2 == 0 else -1) * (1 if side == 0 else -1)
    opts = [o for o in game["options"] if o["team"] == team and o["recovery"] == chain["recovery"]]
    passes = [
        p
        for p in game["passes"]
        if p["team"] == team and p["passer"] == idx % 3 and 4 * p["tick"] >= chain["recovery"]
    ]
    if not opts or not passes:
        return row | {"outcome": "no opening pass"}
    p = passes[0]
    relevant = [o for o in opts if o["tick"] <= p["tick"]]
    if (
        not relevant
        or relevant[-1]["action"] != "pass"
        or relevant[-1]["receiver"] != p["receiver"]
    ):
        return row | {"outcome": "no matching decision"}
    decision = relevant[-1]
    state = states[4 * decision["tick"] - 1]
    xy = state[4:].reshape(6, 3)[:, :2]
    rivals = xy[3:] if side == 0 else xy[:3]
    direct, lane, space = geometry(state[:2], direction, xy[side * 3 + p["receiver"]], rivals)
    row.update(
        passer=idx % 3,
        receiver=p["receiver"],
        slot=p["slot"],
        decision=decision["tick"],
        kick=p["tick"],
        end=p["end"],
        direct=float(direct),
        lane=float(lane),
        space=float(space),
        pass_result=p["result"],
    )
    if direct > protocol["blocked_center_gap_m"]:
        return row | {"outcome": "dribble lane clear"}
    if lane < protocol["minimum_pass_lane_gap_m"] or space < protocol["minimum_receiver_space_m"]:
        return row | {"outcome": "receiver covered"}
    if p["result"] != "complete":
        return row | {"outcome": p["result"]}
    if "goal" not in chain or not chain["passes"] or chain["passes"][0] != p["tick"]:
        return row | {"outcome": "no uninterrupted finish"}
    return row | {"qualified": True, "outcome": "pass goal"}
