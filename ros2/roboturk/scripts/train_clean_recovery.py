"""Use the registered first-touch panels with a learned contested-ball maneuver policy."""

import sys
from pathlib import Path

import train_touch_planner as trainer


def main():
    trainer.WORLD = trainer.WORLD.with_name("ssl_clean_recovery.wbt")
    trainer.CTRL = (
        trainer.WORLD.parent.parent
        / "controllers/roboturk_clean_recovery/roboturk_clean_recovery.py"
    )
    extra = [Path(__file__).resolve(), trainer.ROOT / "src/roboturk_core/roboturk/touch_escape.py"]
    extra.append(trainer.ROOT / "src/roboturk_core/roboturk/early_outlet.py")
    extra.append(trainer.ROOT / "src/roboturk_core/roboturk/roller_rescue.py")
    extra.append(trainer.ROOT / "src/roboturk_core/roboturk/touch_context.py")
    extra.append(trainer.ROOT / "src/roboturk_core/roboturk/finish_touch.py")
    args = sys.argv[1:]
    extra.append(trainer.ROOT / "src/roboturk_core/roboturk/touch_contact.py")
    if "--version" not in args:
        raise SystemExit("Use an explicit checkpoint version starting at 6.")
    version = int(args[args.index("--version") + 1])
    if version < 6:
        raise ValueError("The moving-opponent controller starts at checkpoint 6.")
    lock = trainer.R / f"planned-freeze-{version}.json"
    if args[0] in ("qualify", "control", "historical", "demo"):
        manifest = trainer.read(lock)
        for f in extra:
            key = str(f.relative_to(trainer.ROOT))
            if manifest.get(key) != trainer.sha(f):
                raise ValueError(f"Missing or incorrect frozen source: {key}")
    trainer.main()
    if args[0] == "freeze":
        manifest = trainer.read(lock)
        trainer.save(
            lock, manifest | {str(f.relative_to(trainer.ROOT)): trainer.sha(f) for f in extra}
        )


if __name__ == "__main__":
    main()
