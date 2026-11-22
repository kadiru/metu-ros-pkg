"""Choose a counterattack after a live change of sole possession."""

import math
from .blocked_outlet import finish_motion
from .model import RobotKey, possession, wrap
from .open_play import OpenPlayGame
from .three_match import CFG
from .three_recipient import distance
from .transition_attack import trap, unit, window


class TakeawayObserver:
    """Sole changes and sustained contests use the same durations as the offline audit."""

    def __init__(self):
        self.side = self.last = None
        self.age = self.dual = self.loose = 0
        self.players = set()
        self.last_dual = -1000

    def step(self, held, sample):
        if len({k.team for k in held}) == 2:
            self.dual += 1
            if self.dual == 20 and not self.players:
                self.players = set(held)
            self.last_dual = sample
            self.loose = 0
        else:
            self.dual = 0
            self.loose = self.loose + 1 if not held else 0
            if self.loose >= 100 or sample - self.last_dual > 300:
                self.players.clear()
        key = held[0] if len(held) == 1 else None
        self.age = self.age + 1 if key == self.last else 1
        self.last = key
        if key is None or self.age != 20:
            return None
        contested = key in self.players
        self.players.clear()
        old = self.side
        self.side = key.team
        return key if old is not None and old != key.team and not contested else None


def features(w, actor):
    ball = w.ball
    goal, target, lane = window(w, actor)
    angle = math.atan2(target - ball.y, goal - ball.x)
    gap = min(distance(ball, r) for r in w.robots if r.key.team != actor.key.team)
    return [
        ball.x * w.direction(actor.key.team),
        abs(ball.y),
        gap,
        lane,
        abs(wrap(angle - actor.yaw)),
        math.hypot(ball.vx, ball.vy),
    ]


def select(tree, observation):
    while "action" not in tree:
        tree = tree["left"] if observation[tree["feature"]] <= tree["threshold"] else tree["right"]
    return tree["action"]


# The outlet option retains the existing pass before changing the receiver's finish.
MODES = (
    "incumbent",
    "quick shot",
    "shield then shoot",
    "carry infield",
    "outlet then shoot",
    "moving pivot",
)


def command(w, actor, p, mode, age):
    b = w.ball
    rival = min((r for r in w.robots if r.key.team != actor.key.team), key=lambda r: distance(b, r))
    gx, gy, _ = window(w, actor)
    dx, dy = unit(gx - b.x, gy - b.y)
    ex, ey = unit(b.x - rival.x, b.y - rival.y)
    gap = distance(rival, b)
    risk = max(0, 1 - gap / 0.8)
    # One primitive pivots almost in place. The alternatives carry away from the
    # pressing defender, infield from the touchline, or with the ball's momentum.
    speed, shield, turn, tracking, centering = [
        (0.16, 0.18, 5.8, 0.63, 18),
        (0.55, 1.12, 3.6, 0.69, 19),
        (0.72, 0.68, 4.6, 0.65, 19),
        (0.28, 0.65, 6.3, 0.57, 20),
        (0.85, 0.88, 4.8, 0.53, 21),
    ][mode - 1]
    angle, _, aligned = finish_motion(w, actor, p | dict(shot_clear=0.085), (0, 0))
    vx, vy = speed * dx + shield * risk * ex, speed * dy + shield * risk * ey
    if mode == 2 and age < 13 and gap < 0.44 and not aligned:
        angle = math.atan2(ey, ex)
    if mode == 3:
        vy -= max(-1, min(1, b.y / 1.4)) * 0.82
    if mode == 5 and age < 12:
        vx += 0.52 * b.vx
        vy += 0.52 * b.vy

    # Slow outward translation near either line; retain inward motion and yaw.
    def for_axis(pos, vel, edge):
        return vel * max(0, min(1, (edge - abs(pos)) / 0.35)) if pos * vel > 0 else vel

    vx = for_axis(b.x, vx, 4.25)
    vy = for_axis(b.y, vy, 2.78)
    return trap(
        w,
        actor,
        angle,
        (vx, vy),
        p | dict(turn=turn, tracking=tracking, centering=centering),
        4.0 if aligned else 0,
    )


class TakeawayGame(OpenPlayGame):
    def __init__(self, *args, takeaway_start=None, **kwargs):
        self.takeaway_start = takeaway_start
        self.takeaway_actions = []
        super().__init__(*args, **kwargs)

    def reset(self, team, reason):
        super().reset(team, reason)
        self.takeaway_observer = TakeawayObserver()
        self.counter = None

    def observe_recovery(self):
        skip = self.skip_frames
        super().observe_recovery()
        if skip:
            return
        for j, w in enumerate(self.backend.last_substep_worlds):
            sample = (self.tick - 1) * 4 + j
            held = [r.key for r in w.robots if possession(w, r.key, CFG)]
            if self.counter and any(k.team != self.counter["team"] for k in held):
                self.counter = None
            key = self.takeaway_observer.step(held, sample)
            if key is None:
                continue
            if self.takeaway_start is None and sample < getattr(self, "press_from", 0):
                continue
            tree = self.teams[key.team].get("takeaway_tree")
            if not tree or (self.takeaway_start is not None and sample != self.takeaway_start):
                continue
            obs = features(w, w.robot(key))
            self.counter = dict(
                team=key.team, actor=key.id, sample=sample, x=obs, action=select(tree, obs)
            )
            self.takeaway_actions.append(self.counter)

    def commands(self):
        return self.counter_commands(super().commands())

    def counter_commands(self, out):
        if not self.counter or not self.counter["action"] or self.flight:
            return out
        c = self.counter
        w = self.world
        held = [r for r in w.robots if possession(w, r.key, CFG)]
        if len(held) != 1 or held[0].key.team != c["team"]:
            return out
        actor = held[0]
        if c["action"] == 4 and actor.key == RobotKey(c["team"], c["actor"]):
            return out
        self.owner = actor.key
        self.choice = None
        self.release_plans.pop(self.tick, None)
        out[actor.key] = command(
            w, actor, self.teams[c["team"]], c["action"], self.tick - c["sample"] // 4
        )
        return out
