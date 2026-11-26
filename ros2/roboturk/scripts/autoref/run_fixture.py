"""Run a three-seed passive qualification or an active, scored Webots film."""

import argparse
from contextlib import contextmanager, ExitStack
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

from roboturk.ssl_vision import GameEvent
from verify import ROOT, verify_active, verify_passive


@contextmanager
def service(cmd, cwd, log, marker):
    with log.open("w") as out:
        process = subprocess.Popen(cmd, cwd=cwd, stdout=out, stderr=subprocess.STDOUT)
        try:
            end = time.monotonic() + 20
            while marker not in log.read_text(errors="replace"):
                if process.poll() is not None or time.monotonic() >= end:
                    raise RuntimeError(f"Service failed to start. See {log}")
                time.sleep(0.1)
            yield
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


def match(args, dest, *params):
    cmd = [sys.executable, str(ROOT / "scripts/run_five_roles.py"), str(dest), *params]
    if args.webots:
        cmd.extend(["--webots", str(args.webots)])
    subprocess.run(cmd, cwd=ROOT, check=True)


def observer(args, folder, active=False, *, ci_port=21913, gc_port=21907):
    return service(
        [
            sys.executable,
            str(ROOT / "scripts/autoref/run_tigers.py"),
            str(folder / "observer"),
            "--distribution",
            str(args.distribution),
            "--classes",
            str(args.classes),
            "--java",
            args.java,
            "--mode",
            "ACTIVE" if active else "PASSIVE",
            "--ci-port",
            str(ci_port),
            "--gc-port",
            str(gc_port),
        ],
        ROOT,
        folder / "tigers.log",
        "SSL_READY",
    )


def passive(args, dest):
    for seed, count in ((20300000, 4), (20300001, 3), (20300002, 2)):
        for n in range(1, count + 1):
            match(
                args,
                dest / "tails" / str(seed) / str(n),
                "--seed",
                str(seed),
                "--capture-exit",
                str(n),
            )
        folder = dest / str(seed)
        folder.mkdir()
        with observer(args, folder):
            match(
                args,
                folder / "match",
                "--seed",
                str(seed),
                "--exit-tails",
                str(dest / "tails"),
                "--autoref-ci",
                "127.0.0.1:21913",
                "--ssl-vision",
                "127.0.0.1:21906",
            )
    (dest / "passive.json").write_text(json.dumps(verify_passive(dest), indent=2) + "\n")


def active(args, dest):
    gc = dest / "gc"
    config = gc / "config"
    config.mkdir(parents=True)
    (config / "ssl-game-controller.yaml").write_text(
        "network:\n  publish-address: 127.0.0.1:21903\n  publish-nif: 127.0.0.1\n"
        "game:\n  auto-approve-goals: true\n  default-division: DIV_B\n"
    )
    # The film qualifies goal/out decisions. Keep unrelated contact reports in the log.
    keep = {"POSSIBLE_GOAL", "BALL_LEFT_FIELD_TOUCH_LINE", "BALL_LEFT_FIELD_GOAL_LINE"}
    behavior = {
        name: "BEHAVIOR_ACCEPT" if name in keep else "BEHAVIOR_IGNORE"
        for name in GameEvent.Type.keys()
    }
    behavior.update(
        {name: "BEHAVIOR_ACCEPT_MAJORITY" for name in ("GOAL", "INVALID_GOAL", "AIMLESS_KICK")}
    )
    (config / "engine.yaml").write_text(
        json.dumps({"autoContinue": False, "gameEventBehavior": behavior})
    )
    cmd = [
        args.game_controller,
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
        stack.enter_context(service(cmd, gc, dest / "gc.log", "UI is available"))
        stack.enter_context(observer(args, dest, True))
        stack.enter_context(
            service(
                [
                    "node",
                    str(ROOT / "scripts/autoref/table.mjs"),
                    "ws://127.0.0.1:21981/api/control",
                ],
                ROOT,
                dest / "table.log",
                "TABLE_READY",
            )
        )
        match(
            args,
            dest / "match",
            "--referee",
            "ssl",
            "--referee-address",
            "127.0.0.1",
            "--referee-port",
            "21903",
            "--autoref-ci",
            "127.0.0.1:21913",
            "--ssl-vision",
            "127.0.0.1:21906",
            "--seconds",
            "90",
            "--stop-after-goal-restart",
            "--seed",
            "20300000",
            "--record",
        )

    (dest / "active.json").write_text(json.dumps(verify_active(dest), indent=2) + "\n")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=("passive", "active"))
    p.add_argument("output", type=Path)
    p.add_argument("--distribution", type=Path, required=True)
    p.add_argument("--classes", type=Path, required=True)
    p.add_argument("--java", default="java")
    p.add_argument("--game-controller", default="ssl-game-controller")
    p.add_argument("--webots", type=Path)
    args = p.parse_args()
    for name in ("distribution", "classes", "webots"):
        if (path := getattr(args, name)) is not None:
            setattr(args, name, path.resolve())
    if args.mode == "active":
        gc = shutil.which(args.game_controller)
        if not gc:
            p.error("The active fixture requires --game-controller")
        args.game_controller = str(Path(gc).resolve())
    if not (args.classes / "EventObserver.class").exists():
        p.error("Compile EventObserver.java into --classes before launching")
    dest = args.output.resolve()
    dest.mkdir(parents=True, exist_ok=False)
    (active if args.mode == "active" else passive)(args, dest)


if __name__ == "__main__":
    main()
