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
