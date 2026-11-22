"""Fine-tune on the three old losses and lock a second checkpoint before testing."""

import argparse
import gzip
import json
from pathlib import Path
from roboturk.takeaway_finish import region
from train_takeaway import DEST, ROOT, read, save, sha


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("training", type=Path)
    args = p.parse_args()
    if (DEST / "calibration-freeze.json").exists():
        p.error("Existing calibrated freeze")
    plan = read(DEST / "calibration-protocol.json")
    files = [
        DEST / "freeze.json",
        DEST / "policy.json",
        DEST / "calibration-protocol.json",
        Path(__file__).resolve(),
        ROOT / "scripts/probe_takeaway_options.py",
    ]
    options = {}
    profiles = {}
    records = {}
    for name in plan["searches"]:
        src = args.training / name
        spec = DEST / f"{name}-search.json"
        files.append(spec)
        parameters = read(spec)["profiles"]
        rows = read(src / "options.json")
        save(DEST / f"{name}-returns.json", rows)
        files.append(DEST / f"{name}-returns.json")
        cfg = read(src / "launch.json")
        cfg.pop("root")
        records[name] = dict(
            config=cfg, games=[read(src / f"game-{i:03}.json") for i in range(cfg["cases"])]
        )
        for j, profile in enumerate(parameters):
            subset = [r for r in rows if r["profile"] == j]
            assert len(subset) == 3 and len({(r["seed"], r["sample"]) for r in subset}) == 3
            for r in subset:
                z = region(r["x"], plan["regions"])
                assert z is not None
                wins = sum("goal" in v for v in subset if region(v["x"], plan["regions"]) == z)
                if wins > options.get(z, {}).get("goals", 0):
                    options[z] = dict(search=name, profile=j, goals=wins)
                    profiles[z] = profile
    archive = DEST / "targeted-training.json.gz"
    archive.write_bytes(gzip.compress(json.dumps(records, separators=(",", ":")).encode(), mtime=0))
    save(
        DEST / "calibrated.json",
        read(DEST / "policy.json")
        | dict(takeaway_repairs=dict(regions=plan["regions"], profiles=profiles)),
    )
    save(DEST / "calibration-fit.json", dict(selected=options, reserved_starts_used=0))
    for name in ("takeaway_sprint", "takeaway_finish", "takeaway_release"):
        files.append(ROOT / f"src/roboturk_core/roboturk/{name}.py")
    files.extend([archive, DEST / "calibrated.json", DEST / "calibration-fit.json"])
    save(DEST / "calibration-freeze.json", {str(f.relative_to(ROOT)): sha(f) for f in files})


if __name__ == "__main__":
    main()
