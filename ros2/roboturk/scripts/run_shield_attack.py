"""Run the prespecified native v2 motor search, opening study and independent matches."""

import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
from roboturk.attack_opening import fit, training_example
from audit_recovery_attack import panel
from run_recovery_attack import R as V1, ROOT, read, save
from select_recovery_opening import compare

R = ROOT / "policies/recovery_attack_v2"
WORLD = ROOT / "src/roboturk_ros/webots/worlds/ssl_shield_attack.wbt"
PLAN = R / "protocol.json"


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def freeze_files():
    return [
        PLAN,
        R / "policy.json",
        R / "training.json.gz",
        V1 / "freeze.json",
        Path(__file__).resolve(),
        ROOT / "scripts/audit_shield_attack.py",
        WORLD,
        ROOT / "src/roboturk_core/roboturk/recovery_shield.py",
        ROOT / "src/roboturk_core/roboturk/recovery_camera.py",
        ROOT
        / "src/roboturk_ros/webots/controllers/roboturk_shield_attack/roboturk_shield_attack.py",
    ]


def verify():
    for manifest in [
        V1 / "freeze.json",
        ROOT / "policies/contest_outlet_v2/freeze.json",
        ROOT / "policies/contest_outlet_v1/isolation-freeze.json",
        ROOT / "policies/contest_outlet_v1/freeze.json",
    ]:
        for f, sha in read(manifest).items():
            assert digest(ROOT / f) == sha, f


def main():
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=["train", "qualify", "demo", "ordinary-movie"])
    p.add_argument("folder", type=Path)
    p.add_argument("--qualification", type=Path)
    p.add_argument(
        "--webots",
        type=Path,
        default=ROOT.parent / "runtime/Webots-nightly.app/Contents/MacOS/webots",
    )
    a = p.parse_args()
    plan = read(PLAN)
    verify()
    if a.mode != "train":
        for f, sha in read(R / "freeze.json").items():
            assert digest(ROOT / f) == sha, f
    folder = a.folder.resolve()
    folder.mkdir(exist_ok=False, parents=True)
    binary = a.webots.resolve()
    meta = dict(root=str(ROOT), webots_sha256=digest(binary))

    def setup(d, mode, first, cases, seconds, model, **kwargs):
        d.mkdir(exist_ok=True, parents=True)
        save(
            d / "launch.json",
            dict(
                **meta,
                mode=mode,
                first=first,
                cases=cases,
                seconds=seconds,
                model=str(model),
                **kwargs,
            ),
        )

    def run(d, i=None):
        cfg = read(d / "launch.json")
        env = dict(
            os.environ,
            ATTACK_RUN=str(d),
            PYTHONPATH=str(ROOT / "src/roboturk_core"),
            PATH=str(Path(sys.executable).parent) + os.pathsep + os.environ["PATH"],
        )
        if i is not None:
            env["RECOVERY_CASE"] = str(i)
        cmd = [str(binary), "--batch", "--stdout", "--stderr", "--mode=fast"]
        if not cfg.get("record"):
            cmd += ["--no-rendering", "--minimize"]
        with (d / "webots.log").open("a") as log:
            subprocess.run(
                [*cmd, str(WORLD)], env=env, stdout=log, stderr=subprocess.STDOUT, check=True
            )
        assert (d / f"game-{(cfg['cases'] - 1 if i is None else i):03}.json").exists() and not (
            d / "error.txt"
        ).exists(), d

    def episodes(d):
        return [read(f) for f in sorted(d.glob("game-*.json"))]

    if a.mode == "train":
        if (R / "freeze.json").exists():
            raise ValueError("This version is already frozen")
        controls = plan["controls"]
        d = folder / "motors"
        setup(
            d,
            "motors",
            plan["motors_first"],
            len(controls) * plan["motors_layouts"],
            plan["seconds"],
            V1 / "policy.json",
            controls=controls,
        )
        run(d)
        games = episodes(d)

        def score(rows):
            goals = [
                g["conversions"][0]
                for g in rows
                if g["conversions"] and "goal" in g["conversions"][0]
            ]
            return [
                len(goals),
                sum(bool(e["passes"]) for e in goals),
                sum(e["dribble_m"] >= 0.18 for e in goals),
            ]

        stats = [score([g for g in games if g["control"] == j]) for j in range(len(controls))]
        best = max(range(len(controls)), key=lambda j: stats[j])
        model = read(V1 / "policy.json")
        model["control"] = controls[best]
        motor = folder / "motor.json"
        save(motor, model)
        print("motor returns", stats, "selected", best, flush=True)
        d = folder / "branches"
        setup(
            d,
            "branches",
            plan["branches_first"],
            3 * plan["branches_layouts"],
            plan["seconds"],
            motor,
        )
        run(d)
        branches = episodes(d)
        candidates = [
            compare(branches, n, pn, b)
            for n in (8, 18, 36)
            for pn in (4, 10, 20)
            for b in (0, 3, 6, 9)
        ]
        diverse = [
            row
            for row in candidates
            if min(row["assisted"], row["carried"]) >= 0.08 * row["matched"]
        ]
        selected = max(diverse or candidates, key=lambda r: r["mean_return"])
        model["opening"] = fit(
            [ex for g in branches if (ex := training_example(g))], selected["neighbors"]
        )
        model["opening"].update(
            pass_neighbors=selected["pass_neighbors"],
            pass_success_bonus=selected["pass_success_bonus"],
        )
        opening = folder / "opening.json"
        save(opening, model)
        for name, model_file in [("motor", motor), ("opening", opening)]:
            setup(
                folder / name,
                "trained",
                plan["development_first"],
                plan["development_games"],
                plan["seconds"],
                model_file,
            )
        for i in range(plan["development_games"]):
            for name in ("motor", "opening"):
                run(folder / name, i)
        dev = {k: panel(folder / k) for k in ("motor", "opening")}
        for i in range(plan["development_games"]):
            name = f"trace-{i:03}.npz"
            with np.load(folder / "motor" / name) as x, np.load(folder / "opening" / name) as y:
                np.testing.assert_allclose(x["states"][0], y["states"][0], atol=1e-7, rtol=0)
        choices = [
            k for k, v in dev.items() if min(v["totals"]["assisted"], v["totals"]["carried"]) > 0
        ]
        winner = max(choices or list(dev), key=lambda k: dev[k]["totals"]["first"])
        save(R / "policy.json", read(folder / f"{winner}.json"))
        archive = dict(
            controls=controls,
            motors_launch=read(folder / "motors/launch.json"),
            motor_games=games,
            motor_returns=stats,
            best_motor=best,
            branches_launch=read(d / "launch.json"),
            branches=branches,
            opening_selected=selected,
            candidates=candidates,
            development=dev,
            selected=winner,
        )
        (R / "training.json.gz").write_bytes(
            gzip.compress(json.dumps(archive, separators=(",", ":")).encode(), mtime=0)
        )
        save(
            R / "development.json",
            dict(
                selected=winner,
                panels=dev,
                motor_returns=stats,
                control=controls[best],
                opening_selected=selected,
            ),
        )
        save(R / "freeze.json", {str(f.relative_to(ROOT)): digest(f) for f in freeze_files()})
        print("frozen", winner, {k: v["totals"] for k, v in dev.items()}, flush=True)
    elif a.mode == "qualify":
        counts = dict(
            trained=plan["cases"],
            released=plan["cases"],
            mixed=plan["mixed_cases"],
            ordinary=plan["ordinary_cases"],
        )
        for k, count in counts.items():
            ordinary = k == "ordinary"
            setup(
                folder / k,
                k,
                plan["ordinary_first" if ordinary else "first"],
                count,
                plan["ordinary_seconds" if ordinary else "seconds"],
                R / "policy.json",
            )
        for i in range(plan["cases"]):
            for k, count in counts.items():
                if i < count:
                    run(folder / k, i)
                    print(k, i, flush=True)
    elif a.mode == "demo":
        setup(
            folder,
            "demo",
            plan["demo_first"],
            plan["demo_cases"],
            plan["demo_seconds"],
            R / "policy.json",
            record=True,
        )
        run(folder)
    else:
        assert a.qualification is not None
        panel(a.qualification / "ordinary")
        plays = [
            (bool(e["passes"]), e["dribble_m"] >= 0.18, -e["goal"], g["seed"])
            for g in episodes(a.qualification / "ordinary")
            for e in g["conversions"]
            if "goal" in e
        ]
        seed = max(plays)[-1]
        setup(folder, "ordinary", seed, 1, plan["ordinary_seconds"], R / "policy.json", record=True)
        run(folder)


if __name__ == "__main__":
    main()
