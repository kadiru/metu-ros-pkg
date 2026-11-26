from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from compare_live_takeaways import takeaways  # noqa: E402


@pytest.mark.parametrize("reason", ("clean", "contact", "kick", "restart", "brief", "crash"))
def test_live_takeaway_requires_contested_control_and_rejects_incidental_transfers(reason):
    frames = [dict(restart=0, roles=["Carrier", "Presser"], tackle=[[1, 0]]) for _ in range(15)]
    game = dict(
        dt=0.04,
        physics_dt=0.002,
        frames=frames,
        keys=[["blue", 1], ["yellow", 1]],
        events=[],
    )
    rollers = np.resize(np.repeat([1, 3, 2], [100, 20, 180]), 300)
    contacts = np.zeros(300, dtype=np.uint64)
    crashes = []
    if reason == "contact":
        contacts[104] = 1
    if reason == "kick":
        game["events"].append(dict(tick=5, event="pass"))
    if reason == "restart":
        for f in frames[6:]:
            f["restart"] = 1
    if reason == "brief":
        rollers[230:] = 0
    if reason == "crash":
        crashes.append(dict(tick=5))
    result = takeaways(game, rollers, contacts, crashes)
    if reason in ("kick", "restart", "brief"):
        assert result == []
    else:
        assert len(result) == 1
        assert result[0]["clean"] == (reason == "clean")
        assert result[0]["presser"] and result[0]["trained_tackle"]
