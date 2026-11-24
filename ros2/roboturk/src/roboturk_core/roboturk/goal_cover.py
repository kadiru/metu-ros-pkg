"""Give a vetoed goalmouth contest to the keeper and send the presser out of its lane."""

from dataclasses import replace
import math

from . import field
from .carpet import ROLL_DECEL
from .five_roles import CFG, PENALTY_FRONT, legal_off_ball_target, limit
from .model import possession
from .three_recipient import move

GOAL_CENTER_LIMIT = field.LENGTH / 2 + 0.056
GOAL_CHANNEL_HALF = field.GOAL_WIDTH / 2 - CFG.robot_radius - field.GOAL_WALL - 0.025


def goal_threat(world, team):
    ball = world.ball
    sign = world.direction(team)
    return ball.visible and -CFG.ball_exit_x <= sign * ball.x <= -3.48 and abs(ball.y) < 1.10


def intercept_ball(world, keeper):
    ball = world.ball
    sign = world.direction(keeper.key.team)
    lead = min(0.32, 0.10 + math.hypot(keeper.x - ball.x, keeper.y - ball.y) / 2.4)
    speed = math.hypot(ball.vx, ball.vy)
    duration = min(lead, speed / ROLL_DECEL)
    gain = duration * (1 - ROLL_DECEL * duration / (2 * max(0.001, speed)))
    x = limit(ball.x + ball.vx * gain, CFG.length / 2 + 0.005)
    y = ball.y + ball.vy * gain
    # Cut across the goal mouth from the goal side of a deep ball, with the
    # roller facing upfield for the collect.
    if sign * x < -4.27:
        angle = 0.0 if sign == 1 else math.pi
        target = (
            sign * max(-GOAL_CENTER_LIMIT, sign * x - CFG.possession_x),
            limit(y, GOAL_CHANNEL_HALF - 0.02),
        )
    else:
        angle = math.atan2(y - keeper.y, x - keeper.x)
        target = (
            limit(x - CFG.possession_x * math.cos(angle), 4.35),
            y - CFG.possession_x * math.sin(angle),
        )
    return target, move(keeper, *target, angle, speed=1.16)


def opposing_holder(world, team):
    return next(
        (r for r in world.robots if r.key.team != team and possession(world, r.key, CFG)), None
    )


class GoalCover:
    def __init__(self):
        self.active = {}
        self.applied = []
        self.prepared = []

    def commands(self, world, presser, needed, preparing=False):
        team = presser.key.team
        keeper = next(
            (r for r in world.robots if r.key.team == team and r.key.id == 0 and r.visible), None
        )
        if keeper is None or not goal_threat(world, team):
            self.active.pop(team, None)
            return {}
        # Hand control back to the ordinary carrier behavior for a clean save.
        holders = [r.key for r in world.robots if possession(world, r.key, CFG)]
        if len(holders) == 1 and holders[0].team == team:
            self.active.pop(team, None)
            return {}
        if needed:
            self.active[team] = world.time, presser.key
        previous = self.active.get(team)
        covering = (
            previous is not None
            and previous[1] == presser.key
            and 0 <= world.time - previous[0] <= 0.44
        )
        if not (covering or preparing):
            return {}
        if math.hypot(keeper.x - world.ball.x, keeper.y - world.ball.y) > 1.25:
            return {}
        target, save = intercept_ball(world, keeper)
        if not covering:
            self.prepared.append((world.time, keeper.key))
            return {keeper.key: save}
        # Clear straight upfield first, so the presser does not cross the keeper's
        # lateral approach. Once outside the box, screen the next pass.
        side = math.copysign(1, presser.y - target[1])
        sign = world.direction(team)
        edge = -PENALTY_FRONT + 0.05
        cover_x = sign * max(sign * world.ball.x + 0.46, sign * target[0] + 0.38, edge)
        cover_y = presser.y if sign * presser.x < edge else limit(target[1] + side * 0.55, 2.65)
        cover_x, cover_y = legal_off_ball_target((cover_x, cover_y))
        face = math.atan2(world.ball.y - presser.y, world.ball.x - presser.x)
        cover = replace(move(presser, cover_x, cover_y, face, speed=0.78), dribbler=0)
        self.applied.append((world.time, presser.key, keeper.key))
        return {keeper.key: save, presser.key: cover}
