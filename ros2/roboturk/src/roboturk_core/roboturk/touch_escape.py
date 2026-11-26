"""Choose a first-touch escape from an approaching rival roller in its moving frame."""

from dataclasses import replace
import math

from .carrier_motor import track_roller
from .model import body_vector, possession, world_vector, wrap
from .three_match import CFG
from .three_recipient import distance
from .touch_contact import protect_contact, swing_roller
from .touch_planner import PlannedTouchGame
from .transition_attack import unit


def escape_command(world, actor, rival, p):
    """Estimate time to the opposing mouth and push across its nearest reachable edge."""
    b = world.ball
    x, y = body_vector(b.x - rival.x, b.y - rival.y, rival.yaw)
    vx, vy = body_vector(b.vx - rival.vx, b.vy - rival.vy, rival.yaw)
    vx += rival.omega * y
    vy -= rival.omega * x
    horizon = p["escape_horizon"]
    fx, fy = x + horizon * vx, y + horizon * vy
    if distance(b, rival) > p["escape_range"]:
        return None
    # Only intervene if the swept ball approaches the mouth's enlarged rectangle.
    closest = min(
        max(abs(x + t * vx - 0.115) / 0.035, abs(y + t * vy) / 0.045)
        for t in (0, horizon * 0.5, horizon)
    )
    if closest > p["escape_buffer"]:
        return None
    # A center crossing is safe only if there is time to clear the opposite edge.
    side = math.copysign(1, fy if y * fy < 0 and abs(fy) > 0.075 else y or vy or 1)
    forward = (abs(fx - 0.115) - 0.035) / 0.035
    lateral = (side * fy - 0.045) / 0.045
    # The robot body blocks the rearward path. Prefer the side there.
    front = p["escape_front"] if fx > 0.115 and forward > lateral else 0.0
    ex, ey = unit(front, side)
    ux, uy = world_vector(ex, ey, rival.yaw)
    bx, by = unit(b.x - rival.x, b.y - rival.y)
    ux, uy = unit(ux + p["escape_radial"] * bx, uy + p["escape_radial"] * by)
    gain = p["escape_speed"]
    # Account for movement of the opponent's mouth, including its tangential speed.
    vx = gain * ux + p["escape_follow"] * (rival.vx - rival.omega * (b.y - rival.y))
    vy = gain * uy + p["escape_follow"] * (rival.vy + rival.omega * (b.x - rival.x))
    ease = p["escape_turn"]
    angle = math.atan2(by, bx)
    cmd = track_roller(
        world,
        actor,
        (vx, vy),
        ease * wrap(angle - actor.yaw),
        p
        | {
            "drive": p["escape_drive"],
            "step_v": p["escape_step"],
            "pivot": p["escape_pivot"],
        },
    )
    return cmd, closest


class EscapeGame(PlannedTouchGame):
    def cushion_receiver(self, out):
        runner = self.world.robot(self.runner)
        p = self.teams[runner.key.team]
        if "receive_drive" not in p or not possession(self.world, runner.key, CFG):
            return
        default = out[runner.key]
        if default.kick or not self.shield_commands:
            return
        event = self.shield_commands[-1]
        if event["kick"] != self.delivery["tick"] or event["last"] != self.tick:
            return
        rival = min(
            (r for r in self.world.robots if r.key.team != runner.key.team),
            key=lambda r: distance(r, runner),
        )
        ux, uy = unit(runner.x - rival.x, runner.y - rival.y)
        velocity = p["shield_speed"] * ux, p["shield_speed"] * uy
        motor = p | dict(
            drive=p["receive_drive"],
            step_v=p["receive_step"],
            grip=p["receive_grip"],
            pivot=p["shield_turn"],
        )
        out[runner.key] = track_roller(self.world, runner, velocity, default.omega, motor)

    def commands(self):
        out = super().commands()
        if self.delivery:
            self.cushion_receiver(out)
            return out
        if self.contestants and not self.active:
            tied = [r for r in self.world.robots if possession(self.world, r.key, CFG)]
            if len({r.key.team for r in tied}) == 2:
                for r in tied:
                    p = self.teams[r.key.team]
                    if "duel_scale" not in p or r.key not in self.contestants:
                        continue
                    rival = min(
                        (z for z in tied if z.key.team != r.key.team),
                        key=lambda z: distance(z, self.world.ball),
                    )
                    c = out[r.key]
                    # Retain the learned jaw maneuver while limiting its approach speed.
                    # Sweep toward the rival's nearer side and pivot around our roller.
                    _, offset = body_vector(
                        self.world.ball.x - rival.x, self.world.ball.y - rival.y, rival.yaw
                    )
                    direction = math.copysign(1, offset or math.sin(r.yaw - rival.yaw) or 1)
                    dx, dy = world_vector(0.0, direction * p["duel_sweep"], rival.yaw)
                    sx, sy = body_vector(dx, dy, r.yaw)
                    k, spin = p["duel_scale"], p["duel_turn"]
                    out[r.key] = replace(
                        c,
                        vx=k * c.vx + sx,
                        vy=k * c.vy + sy + (1 - spin) * c.omega * 0.115,
                        omega=spin * c.omega,
                    )
                return out
        if self.flight or self.delivery or not self.first_touches:
            return out
        rec = self.first_touches[-1]
        if rec["last"] != self.tick:
            return out
        held = [r for r in self.world.robots if possession(self.world, r.key, CFG)]
        if len(held) != 1:
            return out
        actor = held[0]
        p = self.teams[actor.key.team]
        if (
            not any(k in p for k in ("escape_horizon", "contact_steps", "swing_speed"))
            or out[actor.key].kick
        ):
            return out
        rival = min(
            (r for r in self.world.robots if r.key.team != actor.key.team),
            key=lambda r: distance(r, self.world.ball),
        )
        if "swing_speed" in p:
            answer = swing_roller(self.world, actor, rival, p)
            metric = "rival_distance"
        elif "contact_steps" in p:
            answer = protect_contact(self.world, actor, rival, out[actor.key], p)
            metric = "predicted_risk_reduction"
        else:
            answer = escape_command(self.world, actor, rival, p)
            metric = "opponent_edge"
        if answer is None:
            return out
        out[actor.key], proximity = answer
        if rec["action"] != "shield":
            rec = rec.copy()
            if self.first_touches[-1]["first"] == self.tick:
                self.first_touches.pop()
            else:
                self.first_touches[-1]["last"] -= 1
            rec.update(first=self.tick, last=self.tick, action="shield")
            self.first_touches.append(rec)
        rec[metric] = proximity
        self.release_plans.pop(self.tick, None)
        self.choice = None
        return out
