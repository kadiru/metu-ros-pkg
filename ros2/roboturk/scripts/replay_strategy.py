#!/usr/bin/env python3
"""Re-run frozen neural inference on recorded native states; no simulated success claim.

The separate geometry auditor does not import runtime metric code. This replay
instead verifies that checkpoint inference reproduces the recorded choices,
latched targets, refreshes and motor-call counts on every control tick.
"""

import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import torch
from audit_strategy import read
from roboturk.continuous import KEYS
from roboturk.model import World, Robot, Ball, Referee
from roboturk.strategy_options import StrategyMatch, MotorBank
from roboturk.strategy_training import load
from roboturk.continuous_safety import filter_commands, CONTRACT as SAFETY

ROOT = Path(__file__).resolve().parents[1]


def world(row, sign):
    return World(
        row[0],
        tuple(Robot(k, *row[5 + i * 6 : 11 + i * 6]) for i, k in enumerate(KEYS)),
        Ball(*row[1:5]),
        Referee.RUNNING,
        sign,
    )


class RecordedStates:
    name = "recorded_native_states_no_physics_simulation"
    contract = "roboturk.webots.closest_kicker.v1"
    command_filter = staticmethod(filter_commands)
    safety_contract = SAFETY

    def __init__(self, record):
        self.record = record
        self.index = 0
        self.resets = [record["initial_snapshot"]] + [
            e["snapshot"] for e in record["option_events"] if e["type"] == "restart_snapshot"
        ]
        self.reset_index = 0
        self.kicks = {}
        for event in record["option_events"]:
            if event["type"] == "applied_kick":
                self.kicks.setdefault(event["step"], []).append(event)

    def reset(self, ignored):
        state = world(self.resets[self.reset_index], self.record["attack_sign"])
        self.reset_index += 1
        return state

    def step(self, commands):
        rows = self.record["physics_trace"][4 * self.index : 4 * self.index + 4]
        self.index += 1
        self.last_substep_worlds = [world(r, self.record["attack_sign"]) for r in rows]
        self.last_kick_events = [
            (
                KEYS[("blue0", "blue1", "yellow0", "yellow1").index(e["robot"])],
                world(e["snapshot"], self.record["attack_sign"]),
                e["speed"],
            )
            for e in self.kicks.get(self.index, [])
        ]
        return self.last_substep_worlds[-1]


def replay(path, blue, yellow):
    torch.set_num_threads(1)
    data = read(path)
    assert data["control"] == "neural"
    models = [load(blue), load(yellow)]
    override = data["motors"].get("pass_override")
    source = None
    if override:
        assert isinstance(override, dict) and override["control"] == "learned frozen pass motor"
        source = Path(override["path"])
        if not source.exists() and "/runs/strategy-v1/" in str(source):
            source = (
                ROOT
                / "policies/strategy_self_play_v1/preparation"
                / str(source).split("/runs/strategy-v1/", 1)[1]
            )
        assert hashlib.sha256(source.read_bytes()).hexdigest() == override["sha256"]
    bank = MotorBank(ROOT, source)
    records = []
    for native in data["matches"]:
        backend = RecordedStates(native)
        match = StrategyMatch(backend, bank, data["duration"])
        match.reset(native["seed"])
        errors = []
        calls = 0
        while not match.finished:
            match.restart_if_needed()
            obs = match.observations()
            actions = np.concatenate(
                [
                    models[0].predict(obs[:2], deterministic=True)[0],
                    models[1].predict(obs[2:], deterministic=True)[0],
                ]
            )
            calls += 2
            choices = actions[:, :10].argmax(1).tolist()
            if choices != native["decision_trace"][match.steps][1:]:
                errors.append(f"neural choice mismatch at step {match.steps + 1}")
                break
            match.step(actions)
        if not errors:
            for field in ("option_events", "strategy_ticks", "motor_calls"):
                actual = match.result()[field]
                # JSON canonicalization also normalizes native numpy scalar types.
                actual = json.loads(
                    json.dumps(
                        actual,
                        default=lambda v: v.item() if isinstance(v, np.generic) else v.tolist(),
                    )
                )
                if actual != native[field]:
                    errors.append(field + " mismatch")
        records.append(
            {
                "seed": native["seed"],
                "passed": not errors,
                "errors": errors,
                "selector_inference_batches": calls,
                "robot_decisions": calls * 2,
                "frozen_motor_calls": dict(match.motor_calls),
            }
        )
        print("Neural replay", native["seed"], not errors, errors, flush=True)
    return {
        "passed": all(r["passed"] for r in records),
        "matches": records,
        "source_sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        "selectors": [
            {"path": str(p), "sha256": hashlib.sha256(Path(p).read_bytes()).hexdigest()}
            for p in (blue, yellow)
        ],
        "motors": bank.descriptors,
        "scope": "Offline inference replay of actual native states, not new physics trials",
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("input", type=Path)
    p.add_argument("--blue", type=Path, required=True)
    p.add_argument("--yellow", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    result = replay(a.input, a.blue, a.yellow)
    a.output.write_text(json.dumps(result, indent=2) + "\n")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
