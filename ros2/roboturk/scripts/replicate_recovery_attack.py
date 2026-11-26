"""Independent larger replication with the original physical audit and frozen teams."""

import argparse
import hashlib
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

from audit_recovery_attack import panel
from run_recovery_attack import R, ROOT, WORLD, read, save


def main():
    p = argparse.ArgumentParser()
    p.add_argument("folder", type=Path)
    p.add_argument("--audit-only", action="store_true")
    p.add_argument(
        "--webots",
        type=Path,
        default=ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots",
    )
    a = p.parse_args()
    folder = a.folder.resolve()
    base = read(R / "protocol.json")
    rep = read(R / "replication-plan.json")
    scale = rep["scale"]
    counts = {
        k: scale * base[v]
        for k, v in (
            ("trained", "cases"),
            ("released", "cases"),
            ("mixed", "mixed_cases"),
            ("ordinary", "ordinary_cases"),
        )
    }
    for name in (
        "recovery_attack_v1/freeze.json",
        "contest_outlet_v2/freeze.json",
        "contest_outlet_v1/freeze.json",
        "contest_outlet_v1/isolation-freeze.json",
    ):
        for file, sha in read(ROOT / "policies" / name).items():
            assert hashlib.sha256((ROOT / file).read_bytes()).hexdigest() == sha, file
    if not a.audit_only:
        folder.mkdir(parents=True, exist_ok=False)
        binary = a.webots.resolve()
        digest = hashlib.sha256(binary.read_bytes()).hexdigest()
        for mode, count in counts.items():
            d = folder / mode
            d.mkdir()
            ordinary = mode == "ordinary"
            save(
                d / "launch.json",
                dict(
                    mode=mode,
                    cases=count,
                    first=rep["ordinary_first" if ordinary else "first"],
                    seconds=base["ordinary_seconds" if ordinary else "seconds"],
                    root=str(ROOT),
                    model=str(R / "policy.json"),
                    webots_sha256=digest,
                ),
            )
        for i in range(max(counts.values())):
            for mode, count in counts.items():
                if i >= count:
                    continue
                d = folder / mode
                env = dict(
                    os.environ,
                    ATTACK_RUN=str(d),
                    RECOVERY_CASE=str(i),
                    PYTHONPATH=str(ROOT / "src/roboturk_core"),
                    PATH=str(Path(sys.executable).parent) + os.pathsep + os.environ["PATH"],
                )
                with (d / "webots.log").open("a") as log:
                    subprocess.run(
                        [
                            str(binary),
                            "--batch",
                            "--stdout",
                            "--stderr",
                            "--mode=fast",
                            "--no-rendering",
                            "--minimize",
                            str(WORLD),
                        ],
                        env=env,
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        check=True,
                    )
                assert (d / f"game-{i:03}.json").exists() and not (d / "error.txt").exists()
                print(mode, i, flush=True)
    panels = {}
    for mode, count in counts.items():
        d = folder / mode
        launch = read(d / "launch.json")
        ordinary = mode == "ordinary"
        assert (launch["mode"], launch["cases"], launch["first"], launch["seconds"]) == (
            mode,
            count,
            rep["ordinary_first" if ordinary else "first"],
            base["ordinary_seconds" if ordinary else "seconds"],
        )
        panels[mode] = panel(d)
    for i in range(counts["released"]):
        name = f"trace-{i:03}.npz"
        with np.load(folder / "released" / name) as release:
            for mode in ["trained", "mixed"] if i < counts["mixed"] else ["trained"]:
                with np.load(folder / mode / name) as other:
                    np.testing.assert_allclose(
                        release["states"][0], other["states"][0], atol=1e-7, rtol=0
                    )
    t, b, h, o = [panels[k]["totals"] for k in ("trained", "released", "mixed", "ordinary")]
    g = base["gates"]
    checks = dict(
        first=t["first"] >= scale * g["first_conversions"],
        improvement=t["first"] - b["first"] >= scale * g["improvement_over_released"],
        mixed=h["mixed_first"] >= scale * g["mixed_trained_conversions"],
        ordinary=o["chains"] >= scale * g["ordinary_recovery_goals"],
        pass_assisted=t["assisted"] >= scale * g["min_pass_assisted"],
        dribble_assisted=t["carried"] >= scale * g["min_dribble_assisted"],
        both_colors=min(t["blue"], t["yellow"]) > 0,
        safety=all(
            v["totals"]["exits"] <= g["max_exits"]
            and v["totals"]["contacts"] / v["totals"]["samples"] <= g["max_contact_fraction"]
            for v in panels.values()
        ),
    )
    save(folder / "audit.json", dict(passed=all(checks.values()), checks=checks, panels=panels))
    print(checks)
    return int(not all(checks.values()))


if __name__ == "__main__":
    raise SystemExit(main())
