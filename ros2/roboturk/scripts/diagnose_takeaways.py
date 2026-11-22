"""Reconstruct the released team's three clean takeaways from the 100 Hz record."""

import argparse
from pathlib import Path
import numpy as np
from train_takeaway import historical, save


def diagnose():
    rows = []
    for f, g, event in historical():
        trace = f.with_name(f.name.replace("game", "trace").replace("json", "npz"))
        s = np.load(trace)["states"]
        p = s[:, 4:].reshape(-1, 6, 3)
        start, end = event["sample"], event["end"]
        i = event["winner"]
        d = s[:, :2] - p[:, i, :2]
        c, t = np.cos(p[:, i, 2]), np.sin(p[:, i, 2])
        own = (abs(d[:, 0] * c + d[:, 1] * t - 0.115) < 0.035) & (
            abs(d[:, 1] * c - d[:, 0] * t) < 0.045
        )
        lost = np.flatnonzero(~own[start : end + 1])
        rival_ids = list(range(3 if i < 3 else 0, 6 if i < 3 else 3))
        opponents = np.linalg.norm(p[start, rival_ids, :2] - s[start, :2], axis=1)
        kicks = [k for k in g["events"] if k["event"] == "kick" and start <= k["tick"] * 4 <= end]
        passes = [item for item in g["passes"] if start <= item["tick"] * 4 <= end]
        rows.append(
            dict(
                source=f.name,
                seed=g["seed"],
                **event,
                recovery_seconds=round((start + 1) / 100, 2),
                elapsed_seconds=(end - start) / 100,
                position_m=s[start, :2].tolist(),
                end_position_m=s[end, :2].tolist(),
                closest_rival_center_m=float(opponents.min()),
                first_carrier_release_or_slip=((int(lost[0]) + start) if len(lost) else None),
                kicks=kicks,
                passes=passes,
                choices=[
                    v
                    for v in g["options"]
                    if v["team"] == event["team"] and start <= v["tick"] * 4 <= end
                ],
            )
        )
    return rows


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    save(parser.parse_args().output, diagnose())
