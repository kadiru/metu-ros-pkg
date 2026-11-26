"""All retained tactical-manager/defense pairings on matched development cases."""

import hashlib
import json
from pathlib import Path
from .decision_play import DecisionEnv, evaluate
from .decision_training import load
from .self_play_training import write


def cross_play(backend, cfg, root, episodes=30, seed=3202000):
    root = Path(root)
    snapshots = json.loads((root / "league.json").read_text())["snapshots"]
    models = [load(b, cfg) for b in snapshots]
    source = Path(__file__).resolve().parents[3]
    h = hashlib.sha256()
    for p in sorted((source / "src").rglob("*")):
        if p.suffix in (".py", ".proto", ".wbt", ".ini"):
            h.update(str(p.relative_to(source)).encode() + b"\0" + p.read_bytes())
    runtime = h.hexdigest()
    directory = root / ("cross-play-" + runtime[:12])
    matrix = []
    for ai, attack in enumerate(models):
        for di, defense in enumerate(models):
            expected = {
                r: (snapshots[di] if r in ("defender", "keeper") else snapshots[ai])[r]
                for r in snapshots[0]
            }
            path = directory / f"attack{ai}-defense{di}.json"
            if path.exists():
                result = json.loads(path.read_text())
                if (
                    result.get("runtime_sha256") != runtime
                    or result["models"] != expected
                    or [e["seed"] for e in result["episodes"]] != list(range(seed, seed + episodes))
                ):
                    raise ValueError("Stale cross-play result")
            else:
                selected = {
                    r: (defense if r in ("defender", "keeper") else attack)[r] for r in snapshots[0]
                }
                env = DecisionEnv(backend, cfg, selected)
                env.defender.sha256 = expected["defender"]["sha256"]
                env.keeper.sha256 = expected["keeper"]["sha256"]
                result = evaluate(env, episodes, seed)
                result.update(
                    models=expected,
                    runtime_sha256=runtime,
                    matchup={"attackers": ai, "defenders": di},
                )
                write(path, result)
            row = {
                "attackers": ai,
                "defenders": di,
                "path": str(path),
                "summary": result["summary"],
            }
            matrix.append(row)
            print("Decision cross-play", ai, di, result["summary"], flush=True)
    write(
        root / "cross-play-matrix.json",
        {"runtime_sha256": runtime, "seed": seed, "episodes_per_pair": episodes, "matrix": matrix},
    )
