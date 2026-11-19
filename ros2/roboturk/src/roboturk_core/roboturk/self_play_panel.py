"""Complete development matrix with one shared scenario set per matchup."""

from pathlib import Path
import json
from .self_play import ROLES, outcomes
from .self_play_training import load_bundle, evaluate_bundle, write


def cross_play(backend, cfg, root, episodes=20, seed=2200200):
    root = Path(root)
    snapshots = json.loads((root / "league.json").read_text())["snapshots"]
    loaded = [load_bundle(b, cfg) for b in snapshots]
    matrix = []
    for ai, attackers in enumerate(loaded):
        for di, defenders in enumerate(loaded):
            path = root / "cross-play" / f"attack{ai}-defense{di}.json"
            expected = {
                r: (snapshots[ai] if r in ("transfer", "finish") else snapshots[di])[r]
                for r in ROLES
            }
            if path.exists():
                result = json.loads(path.read_text())
                if result["models"] != expected or [e["seed"] for e in result["episodes"]] != list(
                    range(seed, seed + episodes)
                ):
                    raise ValueError("Existing cross-play result has different actors or scenarios")
                matrix.append(
                    {
                        "attackers": ai,
                        "defenders": di,
                        "episodes": episodes,
                        **outcomes(result),
                        "path": str(path),
                    }
                )
                print("Retained cross-play", ai, di, flush=True)
                continue
            models = {
                r: (attackers if r in ("transfer", "finish") else defenders)[r] for r in ROLES
            }
            result = evaluate_bundle(backend, cfg, models, episodes, seed)
            result["models"] = expected
            result["matchup"] = {"attackers": ai, "defenders": di}
            result["outcome_summary"] = outcomes(result)
            write(path, result)
            row = {
                "attackers": ai,
                "defenders": di,
                "episodes": episodes,
                **outcomes(result),
                "path": str(path),
            }
            matrix.append(row)
            print("Cross-play", row, flush=True)
    write(
        root / "cross-play-matrix.json",
        {"seed": seed, "episodes_per_pair": episodes, "matrix": matrix},
    )
