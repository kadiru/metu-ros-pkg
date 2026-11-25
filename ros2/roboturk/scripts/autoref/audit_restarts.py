"""Verify the native placements and restarts in an actively officiated Webots game."""

import argparse
import json
from pathlib import Path

import numpy as np

from audit_league import audit, official_events, read_game


def verify_restarts(folder):
    report, _, _ = audit(folder)
    log, states = read_game(folder / "match")
    _, official = official_events(folder)
    frames, events = log["frames"], log["events"]
    centers = [e for e in events if e["event"] == "referee_center"]
    ready = {e["restart"]: e["tick"] for e in events if e["event"] == "formation_ready"}
    kickoff_kicks = [e for e in events if e["event"] == "kickoff_kick"]
    assert len(centers) == len(ready) == len(kickoff_kicks)

    kickoffs = []
    for center, kick in zip(centers, kickoff_kicks, strict=True):
        placed = center["tick"]
        release = ready[center["restart"]]
        shot = kick["tick"]
        team, kicker = log["keys"][kick["actor"]]
        assert team == center["team"] and placed < release < shot
        assert frames[release]["ssl_command"] == "PREPARE_KICKOFF_" + team.upper()
        assert frames[shot]["ssl_command"] == "NORMAL_START"
        error = float(np.linalg.norm(states[release, :2]))
        assert error < 0.004 and np.linalg.norm(states[release, 2:4]) < 0.01
        side = frames[release]["attack_sign"]
        poses = states[release, 4:].reshape(-1, 3)
        for index, (color, _) in enumerate(log["keys"]):
            if index == kick["actor"]:
                continue
            direction = side if color == "blue" else -side
            assert direction * poses[index, 0] < -0.09
            assert np.linalg.norm(poses[index, :2]) >= 0.59
        assert np.linalg.norm(states[shot + 1, :2] - states[shot, :2]) >= 0.05
        kickoffs.append(
            dict(team=team, kicker=kicker, centered_tick=placed + 1, kick_tick=shot, error_m=error)
        )
    for goal in report["goals"]:
        stage = frames[goal["score_tick"]]["stage"]
        restart = next((k for k in kickoffs if k["centered_tick"] > goal["score_tick"]), None)
        if restart and frames[restart["kick_tick"]]["stage"] == stage:
            assert restart["team"].upper() != goal["team"]
            goal["restart"] = restart
        elif frames[goal["score_tick"]]["stage_seconds"] > 0:
            raise AssertionError("No center restart in the scoring half")

    placements = []
    placement_events = [e for e in events if e["event"] == "referee_placement"]
    for index, event in enumerate(placement_events):
        tick, spot, team = event["tick"], event["target"], event["team"]
        assert frames[tick]["ssl_command"] == "BALL_PLACEMENT_" + team.upper()
        assert np.allclose(frames[tick]["target"], spot)
        error = float(np.linalg.norm(states[tick + 1, :2] - spot))
        assert error < 0.004 and np.linalg.norm(states[tick + 1, 2:4]) < 0.01
        assert np.min(np.linalg.norm(states[tick, 4:].reshape(-1, 3)[:, :2] - spot, axis=1)) >= 0.60
        prior = [
            e
            for e in official
            if (e["type"].startswith("BALL_LEFT_FIELD_") or e["type"] == "AIMLESS_KICK")
            and int(e["createdTimestamp"]) / 1e6 <= frames[tick]["vision_time"]
        ]
        kind = "free kick"
        if prior and prior[-1]["type"] != "AIMLESS_KICK":
            boundary = prior[-1]
            if boundary["type"] == "BALL_LEFT_FIELD_TOUCH_LINE":
                kind = "touchline"
                last = boundary["ballLeftFieldTouchLine"]
            else:
                last = boundary["ballLeftFieldGoalLine"]
                sign = frames[tick]["attack_sign"] * (1 if team == "blue" else -1)
                kind = "goal kick" if last["location"].get("x", 0) * sign < 0 else "corner"
            assert last["byTeam"] != team.upper()
            assert abs(abs(spot[1]) - 2.8) < 0.004
            if kind == "goal kick":
                assert abs(abs(spot[0]) - 3.5) < 0.004
            elif kind == "corner":
                assert abs(abs(spot[0]) - 4.3) < 0.004
        command = "DIRECT_FREE_" + team.upper()
        following = placement_events[index + 1] if index + 1 < len(placement_events) else None
        end = following["tick"] if following else len(frames)
        direct = next(
            (i for i in range(tick + 1, end) if frames[i]["ssl_command"] == command), None
        )
        row = dict(kind=kind, team=team, target=spot, error_m=error)
        if direct is None:
            assert following is not None
            row["superseded_by_tick"] = following["tick"]
        else:
            assert np.linalg.norm(states[direct, :2] - spot) < 0.004
            assert any(
                e["type"] == "PLACEMENT_SUCCEEDED"
                and e["placementSucceeded"]["byTeam"] == team.upper()
                and frames[tick]["vision_time"]
                <= int(e["createdTimestamp"]) / 1e6
                <= frames[direct]["vision_time"]
                for e in official
            )
            row["free_kick"] = direct
        placements.append(row)

    report["kickoffs"] = kickoffs
    report["placements"] = placements
    report["body_contact_episodes"] = log["robot_clearance"]["contact_episodes"]
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("match", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(verify_restarts(args.match), indent=2) + "\n")
