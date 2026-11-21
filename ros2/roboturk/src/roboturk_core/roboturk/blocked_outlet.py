"""Shared 3v3 passing decisions and active central cover in contested openings."""

from dataclasses import replace
import math
import numpy as np

from .contest_outlet import duel_layout
from .jaw_recovery import JawGame
from .model import RobotKey, possession, wrap
from .recovery_attack import AttackGame, PossessionEnded
from .three_match import CFG, face
from .three_recipient import Choice, candidates, distance, move
from .transition_attack import carry_velocity, separation, trap, unit, window


def opening(seed, sign):
    rng = np.random.default_rng([seed, 8117])
    w = duel_layout(seed, sign)
    b = replace(w.ball, x=rng.uniform(-0.95, 0.95), y=rng.uniform(-0.55, 0.55))
    wing = rng.choice([-1, 1])
    players = []
    for team in ("blue", "yellow"):
        members = sorted(
            (r for r in w.robots if r.key.team == team), key=lambda r: distance(w.ball, r)
        )
        actor = members[0]
        dx, dy = unit(math.cos(actor.yaw), math.sin(actor.yaw))
        players.append(replace(actor, x=b.x - 0.115 * dx, y=b.y - 0.115 * dy))
        cover, wide = rng.permutation(members[1:])
        direction = w.direction(team)
        ux, uy = unit(-4.5 * direction - b.x, -b.y)
        along = rng.uniform(0.72, 1.32)
        x, y = b.x + along * ux, b.y + along * uy + rng.uniform(-0.10, 0.10)
        players.append(replace(cover, x=x, y=y, yaw=math.atan2(b.y - y, b.x - x)))
        x, y = b.x + direction * rng.uniform(0.5, 1.35), b.y + wing * rng.uniform(0.95, 1.65)
        players.append(replace(wide, x=x, y=y, yaw=math.atan2(b.y - y, b.x - x)))
        wing = -wing
    return replace(w, ball=b, robots=tuple(sorted(players, key=lambda r: r.key)))


def corridor(bx, by, tx, ty, opponents, first=0.05):
    """Minimum perpendicular center clearance on a finite forward segment."""
    ux, uy = unit(tx - bx, ty - by)
    length = math.hypot(tx - bx, ty - by)
    return min(
        (
            abs((x - bx) * uy - (y - by) * ux)
            for x, y in opponents
            if first < (x - bx) * ux + (y - by) * uy < length + 0.04
        ),
        default=2.0,
    )


def geometry(ball, direction, mate, rivals):
    bx, by = ball
    ux, uy = unit(4.53 * direction - bx, -by)
    direct = corridor(bx, by, bx + 1.65 * ux, by + 1.65 * uy, rivals, 0.15)
    space = min(math.dist(mate, rival) for rival in rivals)
    return direct, corridor(bx, by, *mate, rivals), space


def select(w, actor, p, received=False):
    mates = candidates(w, actor)
    rivals = [(r.x, r.y) for r in w.robots if r.key.team != actor.key.team]
    direction = w.direction(actor.key.team)
    b = w.ball
    features = []
    for mate in mates:
        direct, lane, space = geometry((b.x, b.y), direction, (mate.x, mate.y), rivals)
        aim = math.atan2(mate.y - b.y, mate.x - b.x)
        features.append(
            [
                min(0.85, lane),
                min(1.5, space),
                (mate.x - b.x) * direction,
                math.cos(aim - actor.yaw),
                abs(distance(b, mate) - 1.65),
            ]
        )
    scores = np.asarray(features) @ p["receiver"]
    j = int(np.argmax(scores))
    prob = float(1 / (1 + np.exp(np.clip(scores[0] - scores[1], -40, 40))))
    choice = Choice(mates[j].key.id, j, features, prob)
    lane, space = features[j][:2]
    # A continuous comparison allows carrying and shooting to win on clear lanes.
    pass_value = p["bias"] - p["block"] * min(0.75, direct) + p["lane"] * lane + 0.22 * space
    shot_value = (
        0.3
        + 1.25 * min(0.75, direct)
        + p["finish"] * received * min(1.0, window(w, actor)[2] / 0.3)
    )
    carry_value = 0.58 + 0.10 * max(0, 4.5 - direction * b.x) / 5
    values = [pass_value, carry_value, shot_value]
    return choice, ("pass", "dribble", "shoot")[int(np.argmax(values))], values


class BlockedGame(AttackGame):
    def __init__(self, *args, teams, **kwargs):
        self.teams = teams
        super().__init__(*args, **kwargs)

    def reset(self, team, reason):
        super().reset(team, reason)
        if reason == "start":
            self.world = self.backend.reset(opening(self.seed, self.sign))
        self.positions = {}
        w = self.world
        for t in self.teams:
            members = sorted(
                (r for r in w.robots if r.key.team == t), key=lambda r: distance(r, w.ball)
            )
            guard = min(members[1:], key=lambda r: r.x * w.direction(t))
            wide = next(r for r in members[1:] if r != guard)
            self.positions[t] = (guard.key, wide.key, 1 if wide.y >= w.ball.y else -1)

    def formation(self, out, held):
        w, b = self.world, self.world.ball
        for t, (guard, wide, side) in self.positions.items():
            p = self.teams[t]
            direction = w.direction(t)
            ux, uy = unit(-4.53 * direction - b.x, -b.y)
            for key, x, y in [
                (guard, b.x + p["cover"] * ux, b.y + p["cover"] * uy),
                (wide, b.x + direction * p["support_x"], b.y + side * p["support_y"]),
            ]:
                r = w.robot(key)
                if (
                    key in held
                    or key in self.contestants
                    or (
                        self.flight
                        and self.flight["team"] == t
                        and self.flight["receiver"] == key.id
                    )
                ):
                    continue
                if self.flight and self.flight["team"] != t:
                    # Opposing cover can intercept the ball in flight.
                    px, py = b.x + 0.22 * b.vx, b.y + 0.22 * b.vy
                    if math.hypot(r.x - px, r.y - py) < 0.75:
                        x, y = px, py
                out[key] = move(
                    r, float(np.clip(x, -4.12, 4.12)), float(np.clip(y, -2.65, 2.65)), face(r, b)
                )

    def commands(self):
        if self.early_stop and self.terminal_restart:
            raise PossessionEnded
        out = JawGame.commands(self)
        previous = self.active
        self.observe_recovery()
        if self.early_stop and previous and not self.active:
            raise PossessionEnded
        w, b = self.world, self.world.ball
        held = [r for r in w.robots if possession(w, r.key, CFG)]
        self.formation(out, {r.key for r in held})
        if self.flight:
            f = self.flight
            mate = w.robot(RobotKey(f["team"], f["receiver"]))
            if mate in held:
                out[mate.key] = trap(w, mate, mate.yaw, (0, 0), self.teams[mate.key.team])
            self.option = None
            return out
        if len(held) != 1:
            self.option = None
            return out
        actor = held[0]
        team = actor.key.team
        p = self.teams[team]
        rival = min((r for r in w.robots if r.key.team != team), key=lambda r: distance(r, b))
        gx, gy, gap = window(w, actor)
        shot = math.atan2(gy - b.y, gx - b.x)
        escape = separation(w, actor, rival, p["escape"])
        if not self.active or self.active[0] != team:
            if actor.key in self.contestants:
                choice, _, _ = select(w, actor, p)
                mate = w.robot(RobotKey(team, choice.receiver))
                out[actor.key] = trap(w, actor, face(b, mate), escape, p)
            return out
        if (
            not self.option
            or self.option["actor"] != actor.key.id
            or self.tick >= self.option["until"]
        ):
            received = any(
                f["team"] == team and f["result"] == "complete" and 4 * f["tick"] >= self.active[1]
                for f in self.passes
            )
            choice, action, values = select(w, actor, p, received)
            self.option = dict(
                team=team,
                recovery=self.active[1],
                tick=self.tick,
                actor=actor.key.id,
                action=action,
                values=values,
                receiver=choice.receiver,
                until=self.tick + (12 if action == "dribble" else 75),
            )
            self.target_choice = choice
            self.options.append(self.option)
        self.owner = actor.key
        self.choice = None
        action = self.option["action"]
        if action == "pass":
            self.choice = self.target_choice
            mate = w.robot(RobotKey(team, self.choice.receiver))
            out[mate.key] = move(mate, mate.x, mate.y, face(mate, b))
            angle = face(b, mate)
            aligned = (
                abs(wrap(angle - actor.yaw)) < 0.025 and abs(wrap(face(mate, b) - mate.yaw)) < 0.22
            )
            out[actor.key] = trap(w, actor, angle, escape, p, 2.2 if aligned else 0)
        elif action == "shoot":
            aligned = abs(wrap(shot - actor.yaw)) < 0.014
            out[actor.key] = trap(w, actor, shot, escape, p, 4.0 if aligned else 0)
        else:
            velocity = carry_velocity(w, actor, gx, gy, rival, p)
            out[actor.key] = trap(w, actor, shot, velocity, p)
        return out
