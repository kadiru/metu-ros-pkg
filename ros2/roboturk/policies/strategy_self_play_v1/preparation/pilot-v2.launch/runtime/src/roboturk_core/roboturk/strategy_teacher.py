"""Training-only strategic labels; deployment never imports this module."""

import math
import numpy as np
from .model import RobotKey, possession, wrap
from .continuous_teacher import clearance
from .continuous import KEYS
from .tdp_options import BallIntent
from .strategy_options import MODES, intent_values


def actions(match):
    old = match.bank.base_actions(match)
    result = np.zeros((4, 16), np.float32)
    w, cfg = match.world, match.cfg
    for i, k in enumerate(KEYS):
        r, b, s = w.robot(k), w.ball, w.direction(k.team)
        mate = w.robot(RobotKey(k.team, 1 - k.id))
        env = match.options[k]
        mode = int(old[i, :7].argmax())
        held = possession(w, k, cfg)
        mateheld = possession(w, mate.key, cfg)
        ownrange = math.hypot(r.x - b.x, r.y - b.y)
        materange = math.hypot(mate.x - b.x, mate.y - b.y)
        distance = math.hypot(r.x - mate.x, r.y - mate.y)
        opps = [key for key in KEYS if key.team != k.team]
        passed = match.last_team_pass.get(k.team)
        incoming = (
            passed
            and passed["robot"] != f"{k.team}{k.id}"
            and w.time - passed["time"] < 3
            and math.hypot(b.vx, b.vy) > 0.25
        )
        point = (r.x, r.y)
        heading = math.atan2(b.y - r.y, b.x - r.x)
        outlet = (s * 4.5, 0)
        if env and env.kind == "clear" and not env.kicked and env.steps < 180:
            mode = 9
        elif env and env.kind == "receive" and env.dwell >= 25:
            mode = int(old[i, :7].argmax())
        elif env and env.kind == "redirect" and env.kicked:
            mode = int(old[i, :7].argmax())
        elif (
            s * b.x < -2.2
            and ownrange < materange
            and not mateheld
            and w.time - match.memory.last_kick[k] > 1
        ):
            mode = 9
        elif held and w.time - match.memory.last_kick[k] > 1 and 1.0 < distance < 2.3:
            backpass = (
                s * mate.x > 0.7
                and s * (r.x - mate.x) > 0.3
                and abs(
                    wrap(
                        math.atan2(r.y - mate.y, r.x - mate.x)
                        - math.atan2(-mate.y, s * 4.5 - mate.x)
                    )
                )
                < 1.1
            )
            forward = s * (mate.x - r.x) > 0.35 and s * mate.x < 1
            if (backpass or forward) and clearance(w, (b.x, b.y), (mate.x, mate.y), opps) > 0.22:
                mode = 2
        elif (
            (mateheld and 1.0 < distance < 2.3 and ownrange > 1)
            or incoming
            or (
                env and env.kind in ("receive", "redirect") and env.steps < 150 and not env.notified
            )
        ):
            if incoming:
                incoming_heading = math.atan2(-b.vy, -b.vx)
                shot = math.atan2(-r.y, s * 4.5 - r.x)
                speed2 = b.vx * b.vx + b.vy * b.vy
                ahead = ((r.x - b.x) * b.vx + (r.y - b.y) * b.vy) / max(0.01, speed2)
                if (
                    0 < ahead < 1.6
                    and s * r.x > 0.7
                    and abs(wrap(incoming_heading - shot)) < 1.1
                    and clearance(w, (r.x, r.y), (s * 4.5, 0), opps) > 0.22
                ):
                    mode = 8
                else:
                    mode = 7
            else:
                mode = 7
        if mode >= 7:
            kind = MODES[mode]
            if env and env.kind == kind:
                intent = env.intent
            else:
                if kind == "clear":
                    point = (b.x, b.y)
                    candidates = [(s * -0.6, s * y) for y in (-1.7, 1.7)]
                    outlet = max(candidates, key=lambda out: clearance(w, point, out, opps))
                    heading = math.atan2(outlet[1] - b.y, outlet[0] - b.x)
                elif kind == "receive":
                    heading = math.atan2(b.y - r.y, b.x - r.x)
                    # Hold the existing robot pose while facing the announced ball.
                    point = (r.x + 0.115 * math.cos(heading), r.y + 0.115 * math.sin(heading))
                else:
                    heading = math.atan2(-r.y, s * 4.5 - r.x)
                    desired = (r.x + 0.115 * math.cos(heading), r.y + 0.115 * math.sin(heading))
                    speed2 = b.vx * b.vx + b.vy * b.vy
                    t = ((desired[0] - b.x) * b.vx + (desired[1] - b.y) * b.vy) / max(0.01, speed2)
                    point = (b.x + t * b.vx, b.y + t * b.vy)
                intent = BallIntent(kind, point, heading, outlet)
            result[i, 10:] = intent_values(w, k, intent)
        result[i, :10] = -0.8
        result[i, mode] = 0.8
    return np.clip(result, -1, 1)
