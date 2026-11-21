"""Pass-motor diagnostic and training labels; never a deployment fallback."""

import math
import numpy as np
from .model import Command, body_vector, wrap, possession
from .continuous import KEYS, encode


def pass_command(match, key):
    w, cfg = match.world, match.cfg
    r, b = w.robot(key), w.ball
    mate = next(p for p in w.robots if p.key.team == key.team and p.key != key)
    angle = wrap(math.atan2(mate.y - b.y, mate.x - b.x) - r.yaw)
    omega = float(np.clip(2.2 * angle, -1.1, 1.1))
    bx, by = body_vector(b.x - r.x, b.y - r.y, r.yaw)
    held = possession(w, key, cfg)
    contested = any(possession(w, p.key, cfg) for p in w.robots if p.key.team != key.team)
    if held and abs(angle) < 0.035 and (match.memory.dwell[key] >= 20 or contested):
        return Command(kick=2.2)
    return Command(3 * (bx - cfg.possession_x), -cfg.possession_x * omega + 3 * by, omega, 1)


class DiagnosticPassBank:
    """Explicit training-only motor ablation; wraps the otherwise frozen bank."""

    def __init__(self, bank):
        self.bank = bank
        self.skills = bank.skills
        self.descriptors = {**bank.descriptors, "pass_override": "analytic diagnostic only"}

    def base_actions(self, match):
        result = self.bank.base_actions(match)
        for i, key in enumerate(KEYS):
            result[i, 15:19] = encode(2, pass_command(match, key))[15:19]
        return result


def observation(match, key):
    """Robot-relative geometry and contact history, with no strategic label."""
    w, cfg = match.world, match.cfg
    r, b = w.robot(key), w.ball
    mate = next(p for p in w.robots if p.key.team == key.team and p.key != key)
    bx, by = body_vector(b.x - r.x, b.y - r.y, r.yaw)
    bvx, bvy = body_vector(b.vx, b.vy, r.yaw)
    rvx, rvy = body_vector(r.vx, r.vy, r.yaw)
    angle = wrap(math.atan2(mate.y - b.y, mate.x - b.x) - r.yaw)
    return np.array(
        [
            (bx - cfg.possession_x) / 0.1,
            by / 0.1,
            bvx / 3,
            bvy / 3,
            angle / math.pi,
            math.sin(angle),
            math.cos(angle),
            np.clip(angle / 0.1, -3, 3),
            math.hypot(mate.x - b.x, mate.y - b.y) / 5,
            min(40, match.memory.dwell[key]) / 20,
            float(possession(w, key, cfg)),
            float(any(possession(w, p.key, cfg) for p in w.robots if p.key.team != key.team)),
            rvx / 1.2,
            rvy / 1.2,
            r.omega / 4,
            math.hypot(mate.vx, mate.vy),
        ],
        np.float32,
    )


def network():
    import torch

    return torch.nn.Sequential(
        torch.nn.Linear(16, 64),
        torch.nn.Tanh(),
        torch.nn.Linear(64, 64),
        torch.nn.Tanh(),
        torch.nn.Linear(64, 4),
    )


class LearnedPass:
    """Frozen learned motor replacement; all strategic choices remain external."""

    def __init__(self, path):
        import hashlib
        import json
        from pathlib import Path
        import torch

        path = Path(path)
        meta = json.loads(path.with_suffix(".json").read_text())
        assert meta["contract"] == "roboturk.strategy_pass.v1"
        assert meta["checkpoint_sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
        self.model = network()
        self.model.load_state_dict(torch.load(path, map_location="cpu", weights_only=True))
        self.model.eval()
        self.descriptor = {
            "path": str(path.resolve()),
            "sha256": meta["checkpoint_sha256"],
            "contract": meta["contract"],
            "control": "learned frozen pass motor",
        }

    def predict(self, match):
        import torch

        with torch.no_grad():
            x = torch.as_tensor(np.stack([observation(match, k) for k in KEYS]))
            return self.model(x).clamp(-1, 1).numpy()
