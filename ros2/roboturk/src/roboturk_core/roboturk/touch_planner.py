"""Score short candidate motions by their predicted roller and opponent clearances."""

import math
from .model import RobotKey, body_vector, possession, world_vector, wrap
from .pressure_receive import PressureGame
from .carrier_motor import protected_motion
from .recovery_touch import RecoveryTouchGame
from .three_match import CFG, face
from .three_recipient import distance, move
from .transition_attack import trap, unit


def outlook(w, actor, rival, cmd, p):
    """Short kinematic forecast with a damped roller; use only observed opponent motion."""
    x, y, yaw = actor.x, actor.y, actor.yaw
    bx, by, bvx, bvy = w.ball.x, w.ball.y, w.ball.vx, w.ball.vy
    rx, ry, ra = rival.x, rival.y, rival.yaw
    peak = threat = 0.0
    dt = 0.02
    for _ in range(p["steps"]):
        vx, vy = world_vector(cmd.vx, cmd.vy, yaw)
        scale = min(1.0, 1.2 / max(0.001, math.hypot(vx, vy)))
        vx *= scale
        vy *= scale
        # Approximate the match's robot separation constraint during close contact.
        nx, ny = unit(x - rx, y - ry)
        gap = math.hypot(x - rx, y - ry)
        inward = (rival.vx - vx) * nx + (rival.vy - vy) * ny - max(0, (gap - 0.195) * 0.8 / 0.04)
        if inward > 0 and gap < 0.33:
            vx += 0.5 * inward * nx
            vy += 0.5 * inward * ny
        mx, my = x + 0.115 * math.cos(yaw), y + 0.115 * math.sin(yaw)
        fx = 8 * (mx - bx) + 0.15 * (vx - bvx)
        fy = 8 * (my - by) + 0.15 * (vy - bvy)
        fscale = min(1.0, 0.4 / max(0.0001, math.hypot(fx, fy)))
        bvx += fscale * fx / 0.046 * dt
        bvy += fscale * fy / 0.046 * dt
        bx += dt * bvx
        by += dt * bvy
        x += dt * vx
        y += dt * vy
        yaw += dt * cmd.omega
        rx += dt * rival.vx
        ry += dt * rival.vy
        ra += dt * rival.omega
        ax, ay = body_vector(bx - x, by - y, yaw)
        ox, oy = body_vector(bx - rx, by - ry, ra)
        edge = max(abs(ax - 0.115) / 0.035, abs(ay) / 0.045)
        alien = max(abs(ox - 0.115) / 0.035, abs(oy) / 0.045)
        peak = max(peak, edge)
        threat = max(threat, max(0, p["buffer"] - alien))
    # The margin prevents a switch back to a fast pivot when the ball grazes either edge.
    return (
        (
            p["own_cost"] * max(0.0, peak - p["own_margin"]) ** 2
            + p["enemy_cost"] * threat**2
            + 2.5 * max(0.0, abs(by) - 2.5)
        ),
        peak,
        threat,
    )


class PlannedTouchGame(RecoveryTouchGame):
    def commands(self):
        held = [r for r in self.world.robots if possession(self.world, r.key, CFG)]
        if len(held) != 1 or "own_cost" not in self.teams[held[0].key.team]:
            return super().commands()
        out = PressureGame.commands(self)
        if self.flight or self.delivery or not (self.active or self.contestants):
            return out
        w = self.world
        b = w.ball
        actor = held[0]
        team = actor.key.team
        p = self.teams[team]
        if self.active:
            t, start = self.active
            if t != team or any(f["team"] == t and 4 * f["tick"] >= start for f in self.passes):
                return out
        elif actor.key not in self.contestants:
            return out
        rival = min((r for r in w.robots if r.key.team != team), key=lambda r: distance(b, r))
        option = self.option
        mate = (
            w.robot(RobotKey(team, option["receiver"]))
            if option and option["actor"] == actor.key.id
            else max(
                (r for r in w.robots if r.key.team == team and r.key != actor.key),
                key=lambda r: distance(r, rival),
            )
        )
        default = out[actor.key]
        previous = self.first_touches[-1] if self.first_touches else {}
        last = (
            previous.get("action")
            if previous.get("team") == team
            and previous.get("actor") == actor.key.id
            and previous.get("last") == self.tick - 1
            else None
        )
        ready = bool(default.kick and self.choice)
        action = "sidestep" if option and option["action"] == "dribble" else "release"
        chosen = default
        if p.get("cushion") and not ready:
            chosen, action, edge, threat = protected_motion(
                w, actor, rival, mate, default, last, p, option and option["action"] == "dribble"
            )
            cost = best = 0.0
        else:
            cost, edge, threat = outlook(w, actor, rival, default, p)
            best = cost
        if (
            not p.get("cushion")
            and not ready
            and (cost > p["gate"] or (last in ("shield", "sidestep") and cost > p["gate"] * 0.40))
        ):
            away = unit(b.x - rival.x, b.y - rival.y)
            left = (-math.sin(rival.yaw), math.cos(rival.yaw))
            lateral = (b.x - rival.x) * left[0] + (b.y - rival.y) * left[1]
            drift = (b.vx - rival.vx) * left[0] + (b.vy - rival.vy) * left[1]
            sign = 1 if lateral + 0.08 * drift >= 0 else -1
            angle = face(b, mate)
            back = math.atan2(away[1], away[0])
            speed = p["cruise"]
            peak_rate = max(0.6, p["pivot"] * (1 - 0.6 * min(1.0, math.hypot(b.vx, b.vy))))
            motor = p | dict(centering=p["grip"], tracking=p["follow"], turn=peak_rate)
            # Both roller edges compete. The opposite edge sometimes opens as the rival turns.
            candidates = [("shield", back, away, speed), ("shield", actor.yaw, away, 0.45 * speed)]
            for side in (sign, -sign):
                direction = unit(0.55 * away[0] + side * left[0], 0.55 * away[1] + side * left[1])
                candidates.append(("sidestep", angle, direction, speed))
                candidates.append(("sidestep", actor.yaw, direction, 1.12 * speed))
            for mode, heading, direction, velocity in candidates:
                cmd = trap(
                    w, actor, heading, (velocity * direction[0], velocity * direction[1]), motor
                )
                value, _, _ = outlook(w, actor, rival, cmd, p)
                value += p["gate"] + p["align_cost"] * abs(wrap(angle - heading))
                if last == mode:
                    value -= p["stick"]
                if value < best:
                    best = value
                    chosen = cmd
                    action = mode
        if chosen is not default:
            out[actor.key] = chosen
            self.release_plans.pop(self.tick, None)
            self.choice = None
        if action == "sidestep" and self.active and option and option["action"] == "dribble":
            # Have the outlet widen its angle while the carrier moves across the close guard.
            _, _, flank = self.positions[team]
            ux, uy = unit(mate.x - b.x, mate.y - b.y)
            side = 1 if uy * ux > 0 else -1 if uy * ux < 0 else flank
            x = max(-4.1, min(4.1, mate.x - side * 0.32 * uy))
            y = max(-2.58, min(2.58, mate.y + side * 0.32 * ux))
            out[mate.key] = move(mate, x, y, face(mate, b), speed=p["run_speed"])
            option["until"] = min(option["until"], self.tick + 3)
        if last != action:
            self.first_touches.append(
                dict(
                    first=self.tick,
                    team=team,
                    actor=actor.key.id,
                    action=action,
                    default_edge=edge,
                    default_threat=threat,
                    selected_cost=best,
                )
            )
        self.first_touches[-1]["last"] = self.tick
        return out
