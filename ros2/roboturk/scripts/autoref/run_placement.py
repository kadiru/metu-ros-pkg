"""Run a native placement batch with the upstream Game Controller and TIGERs."""

import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import shutil

from roboturk.ssl_vision import GameEvent
from run_fixture import ROOT, service, observer, match
from verify import load_events


CRASHES = {"BOT_CRASH_UNIQUE", "BOT_CRASH_DRAWN"}


def audit(dest, strict=False):
    trials = json.loads((dest / "match/placement.json").read_text())
    store = dest / "gc/config/state-store.json.stream"
    changes = [json.loads(line)["change"] for line in store.read_text().splitlines()]
    official = [c["addGameEventChange"]["gameEvent"] for c in changes if "addGameEventChange" in c]
    emitted = load_events(dest / "tigers.log")
    used = set()
    for row in trials:
        detected = [
            p
            for stamp, _, p in emitted
            if row["vision_start"] - 0.001 <= stamp <= row["vision_end"] + 0.001
        ]
        row["observer_events"] = [
            p for p in detected if p["type"] in (KEEP | CRASHES if strict else KEEP)
        ]
        additions = [
            e
            for e in official
            if e.get("id") in row["event_ids"] and "TIGERs AutoRef" in e.get("origin", [])
        ]
        row["controller_events"] = additions
        own = row["team"].upper()
        ids = {
            e["id"]
            for e in additions
            if e["type"] == "PLACEMENT_SUCCEEDED" and e["placementSucceeded"]["byTeam"] == own
        }
        if used & ids:
            raise RuntimeError("An accepted event was reused across fixtures")
        used.update(ids)
        row["accepted"] = (
            bool(ids)
            and any(e.get("placementSucceeded", {}).get("byTeam") == own for e in detected)
            and row.get("placed_seconds", 100) <= 30
        )
    interference = sum(1 for _, k, _ in emitted if k == "bot_interfered_placement")
    report = dict(
        trials=trials,
        accepted=sum(r["accepted"] for r in trials),
        interference=interference,
        emitted_placements=sum(k == "placement_succeeded" for _, k, _ in emitted),
        total=len(trials),
    )
    if strict:
        report["crashes"] = sum(p["type"] in CRASHES for _, _, p in emitted)
        report["controller_crashes"] = sum(e["type"] in CRASHES for e in official)
        report["clean"] = sum(
            row["accepted"]
            and not any(
                e["type"] in CRASHES | {"BOT_INTERFERED_PLACEMENT"} for e in row["observer_events"]
            )
            for row in trials
        )
    (dest / "audit.json").write_text(json.dumps(report, indent=2) + "\n")
    print({k: v for k, v in report.items() if k != "trials"}, flush=True)


KEEP = {
    "PLACEMENT_SUCCEEDED",
    "BOT_INTERFERED_PLACEMENT",
    "BALL_LEFT_FIELD_TOUCH_LINE",
    "PLACEMENT_FAILED",
}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("spec", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--game-controller", default=ROOT.parent / "runtime/ssl-game-controller")
    p.add_argument("--distribution", type=Path, default=ROOT.parent / "runtime/autoReferee")
    p.add_argument("--classes", type=Path, default=ROOT / "runs/autoref-v1")
    p.add_argument(
        "--java", default=str(ROOT.parent / "runtime/java-autoref/Contents/Home/bin/java")
    )
    p.add_argument("--webots", type=Path)
    args = p.parse_args()
    dest = args.output.resolve()
    dest.mkdir(parents=True, exist_ok=False)
    config = dest / "gc/config"
    config.mkdir(parents=True)
    (config / "ssl-game-controller.yaml").write_text(
        "network:\n  publish-address: 127.0.0.1:21903\n  publish-nif: 127.0.0.1\ngame:\n  default-division: DIV_B\n"
    )
    spec = json.loads(args.spec.read_text())
    strict = spec.get("adjudicate_crashes", False)
    enabled = KEEP | CRASHES if strict else KEEP
    behavior = {
        name: "BEHAVIOR_ACCEPT" if name in enabled else "BEHAVIOR_IGNORE"
        for name in GameEvent.Type.keys()
    }
    (config / "engine.yaml").write_text(
        json.dumps(dict(autoContinue=False, gameEventBehavior=behavior))
    )
    cmd = [
        shutil.which(str(args.game_controller)),
        "-address",
        "127.0.0.1:21981",
        "-autorefAddress",
        "127.0.0.1:21907",
        "-teamAddress",
        "127.0.0.1:21908",
        "-remoteControlAddress",
        "127.0.0.1:21911",
        "-ciAddress",
        "127.0.0.1:21909",
    ]
    with ExitStack() as stack:
        stack.enter_context(service(cmd, dest / "gc", dest / "gc.log", "UI is available"))
        stack.enter_context(observer(args, dest, True))
        params = [
            "--placement-batch",
            str(args.spec.resolve()),
            "--referee",
            "ssl",
            "--referee-address",
            "127.0.0.1",
            "--referee-port",
            "21903",
            "--autoref-ci",
            "127.0.0.1:21913",
        ]
        if spec["mode"] == "demo":
            params.append("--record")
        match(args, dest / "match", *params)
    audit(dest, strict)


if __name__ == "__main__":
    main()
