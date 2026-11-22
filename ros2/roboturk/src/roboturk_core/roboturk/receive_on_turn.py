"""Receive on the half-turn, keeping the ball behind the body while facing the shot."""

import math

from .carrier_motor import track_roller
from .finish_touch import CleanRecoveryGame
from .model import possession, wrap
from .roller_rescue import rescue_motion
from .three_match import CFG
from .three_recipient import distance
from .transition_attack import unit, window


def shield_heading(away, shot, angle_limit):
    return away + max(-angle_limit, min(angle_limit, wrap(shot - away)))


class HalfTurnGame(CleanRecoveryGame):
    def commands(self):
        out = super().commands()
        if self.delivery:
            runner = self.world.robot(self.runner)
            guard = self.teams[runner.key.team].get("catch_guard")
            cmd = out[runner.key]
            if guard and not cmd.kick and possession(self.world, runner.key, CFG):
                marker = min(
                    (r for r in self.world.robots if r.key.team != runner.key.team),
                    key=lambda r: distance(r, self.world.ball),
                )
                adjustment = rescue_motion(self.world, runner, marker, cmd, guard, 100)
                if adjustment is not None:
                    out[runner.key] = adjustment[0]
                    return out
        if not self.delivery or not self.shield_commands:
            return out
        event = self.shield_commands[-1]
        if event["kick"] != self.delivery["tick"] or event["last"] != self.tick:
            return out
        w = self.world
        actor = w.robot(self.runner)
        p = self.teams[actor.key.team]
        if "half_angle" not in p or not possession(w, actor.key, CFG):
            return out
        rival = min(
            (r for r in w.robots if r.key.team != actor.key.team), key=lambda r: distance(r, actor)
        )
        ux, uy = unit(actor.x - rival.x, actor.y - rival.y)
        away = math.atan2(uy, ux)
        gx, gy, _ = window(w, actor)
        shot = math.atan2(gy - w.ball.y, gx - w.ball.x)
        # Stay within the protective cone while reducing the turn still needed to shoot.
        age = self.tick - event["first"]
        fade = min(1.0, max(0.0, (age - p["half_delay"] + 1) / p["half_ramp"]))
        yaw = shield_heading(away, shot, fade * p["half_angle"])
        blend = p["half_carry"] * fade
        ex, ey = unit(
            (1 - blend) * ux + blend * math.cos(yaw), (1 - blend) * uy + blend * math.sin(yaw)
        )
        omega = p["half_gain"] * wrap(yaw - actor.yaw)
        motor = p | dict(
            drive=p["half_drive"],
            step_v=p["half_step"],
            grip=p["half_grip"],
            pivot=p["shield_turn"],
            cap=1.16,
        )
        if age < p.get("settle_ticks", 0):
            speed = math.hypot(w.ball.vx, w.ball.vy)
            if speed > p["settle_speed"] and distance(rival, actor) > p["settle_space"]:
                # Absorb a fast arrival before exposing the ball to a full-speed turn.
                # A marker already at the body leaves no room to delay the shield.
                target = (
                    p["settle_roll"] * w.ball.vx + 0.18 * ux,
                    p["settle_roll"] * w.ball.vy + 0.18 * uy,
                )
                out[actor.key] = track_roller(
                    w,
                    actor,
                    target,
                    p["settle_spin"] * omega,
                    motor | {"drive": 0.45, "step_v": 0.28},
                )
                return out
        out[actor.key] = track_roller(
            w, actor, (p["shield_speed"] * ex, p["shield_speed"] * ey), omega, motor
        )
        return out
