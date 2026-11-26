"""Run independent Webots matches with a separate controller port per case."""

from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess
import sys
from zlib import crc32


def launch_cases(folder, cfg, exe, workers, resume=False):
    folder.mkdir(parents=True, exist_ok=resume)
    launch = folder / "launch.json"
    if launch.exists():
        if not resume or json.loads(launch.read_text()) != cfg:
            raise ValueError("The existing schedule or checkpoint differs")
    else:
        launch.write_text(json.dumps(cfg, indent=2) + "\n")
    root = Path(cfg["root"])
    world = root / cfg["world"]
    first_port = 10000 + crc32(str(folder).encode()) % 45000

    def play(i):
        game = folder / f"game-{i:03}.json"
        trace = folder / f"trace-{i:03}.npz"
        if resume and game.exists() and trace.exists():
            return
        env = dict(
            os.environ,
            ATTACK_RUN=str(folder),
            RECOVERY_CASE=str(i),
            PYTHONPATH=str(root / "src/roboturk_core"),
            PATH=str(Path(sys.executable).parent) + os.pathsep + os.environ["PATH"],
        )
        log = folder / f"webots-{i:03}.log"
        for attempt in range(3):
            with log.open("w") as stream:
                result = subprocess.run(
                    [
                        str(exe),
                        f"--port={first_port + i + 1049 * attempt}",
                        "--batch",
                        "--mode=fast",
                        "--no-rendering",
                        "--minimize",
                        "--stdout",
                        "--stderr",
                        str(world),
                    ],
                    env=env,
                    stdout=stream,
                    stderr=subprocess.STDOUT,
                )
            if result.returncode == 0 and game.exists() and trace.exists():
                return
            if "failed to open TCP server" not in log.read_text():
                break
        raise RuntimeError(f"Webots case {i} failed. See {log}")

    with ThreadPoolExecutor(max_workers=workers) as pool:
        list(pool.map(play, range(cfg["cases"])))
