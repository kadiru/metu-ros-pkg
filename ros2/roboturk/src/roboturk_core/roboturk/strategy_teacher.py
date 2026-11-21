"""Training-only strategic labels; deployment never imports this module."""

import math
import numpy as np
from .model import RobotKey, possession, wrap
from .continuous_teacher import ContinuousTeacher, clearance
from .continuous import KEYS
from .tdp_options import BallIntent
from .strategy_options import MODES, ACTIONS, intent_values


def actions(match):
    # Teach decisions from the explicit coach, not the previous selector's
    # mistakes. The motor commands still come from the frozen neural bank.
    old = ContinuousTeacher().actions(match)
    result = np.zeros((4, ACTIONS), np.float32)
    w, cfg = match.world, match.cfg
    for i, k in enumerate(KEYS):
        r, b, s = w.robot(k), w.ball, w.direction(k.team)
        mate = w.robot(RobotKey(k.team, 1 - k.id))
        env = match.options[k]
        mode = int(old[i, :7].argmax())
        held = possession(w, k, cfg)
        mateheld = possession(w, mate.key, cfg)
        opps = [key for key in KEYS if key.team != k.team]
        enemyheld = any(possession(w, key, cfg) for key in opps)
        ownrange = math.hypot(r.x - b.x, r.y - b.y)
        materange = math.hypot(mate.x - b.x, mate.y - b.y)
        distance = math.hypot(r.x - mate.x, r.y - mate.y)
        passed = match.last_team_pass.get(k.team)
        speed2 = b.vx * b.vx + b.vy * b.vy
        ahead = ((r.x - b.x) * b.vx + (r.y - b.y) * b.vy) / max(0.01, speed2)
        incoming = bool(
            passed
            and passed["robot"] != f"{k.team}{k.id}"
            and w.time - passed["time"] < 3
            and speed2 > 0.0625
        )
        shot = math.atan2(-r.y, s * 4.5 - r.x)
        can_redirect = bool(
            incoming
            and passed["mode"] == "pass"
            and 0 < ahead < 1.8
            and s * r.x > 0.7
            and abs(wrap(math.atan2(-b.vy, -b.vx) - shot)) < 1.1
            and clearance(w, (r.x, r.y), (s * 4.5, 0), opps) > 0.22
        )
        active = env and not env.kicked and env.steps < 180
        if active and env.kind == "clear":
            mode = 9
        elif active and env.kind == "redirect":
            mode = 8
        elif active and env.kind == "receive" and env.dwell < 25:
            mode = 8 if can_redirect else 7
        elif env and (env.kicked or env.dwell >= 25 or env.steps >= 180):
            pass
        elif incoming and 0 < ahead < 2 and ownrange < materange:
            mode = 8 if can_redirect else 7
        elif (
            s * b.x < -2.2
            and ownrange < materange
            and not mateheld
            and not enemyheld
            and w.time - match.memory.last_kick[k] > 1
        ):
            mode = 9
        elif held and w.time - match.memory.last_kick[k] > 1 and 1 < distance < 2.3:
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
        elif (mateheld and 1 < distance < 2.3 and ownrange > 1) or (
            s * b.x < -2.2 and materange < 0.6 and ownrange > 1 and not enemyheld
        ):
            mode = 7
        if mode >= 7:
            kind = MODES[mode]
            if env and env.kind == kind:
                intent = env.intent
            else:
                heading = math.atan2(b.y - r.y, b.x - r.x)
                outlet = (s * 4.5, 0)
                point = (r.x + 0.115 * math.cos(heading), r.y + 0.115 * math.sin(heading))
                if kind == "clear":
                    point = (b.x, b.y)
                    outlet = max(
                        [(s * -0.6, s * y) for y in (-1.7, 1.7)],
                        key=lambda out: clearance(w, point, out, opps),
                    )
                    heading = math.atan2(outlet[1] - b.y, outlet[0] - b.x)
                elif kind == "redirect":
                    heading = shot
                    point = (r.x + 0.115 * math.cos(heading), r.y + 0.115 * math.sin(heading))
                intent = BallIntent(kind, point, heading, outlet)
            refresh = (
                kind in ("receive", "redirect")
                and incoming
                and not held
                and not (env and env.ever_contact)
                and ahead > 0
            )
            if refresh:
                t = ((intent.point[0] - b.x) * b.vx + (intent.point[1] - b.y) * b.vy) / max(
                    0.01, speed2
                )
                point = (b.x + t * b.vx, b.y + t * b.vy)
                heading = math.atan2(-b.vy, -b.vx) if kind == "receive" else shot
                intent = BallIntent(kind, point, heading, intent.outlet)
            result[i, 10:16] = intent_values(w, k, intent)
            result[i, 16] = 0.8 if refresh else -0.8
        result[i, :10] = -0.8
        result[i, mode] = 0.8
    return np.clip(result, -1, 1)
