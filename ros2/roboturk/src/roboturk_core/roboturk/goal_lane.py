"""Break a shared keeper grip toward the field and give the presser a clear outlet."""

import math
from dataclasses import replace

from .five_roles import CFG, PENALTY_FRONT, legal_off_ball_target
from .goal_cover import opposing_holder
from .model import possession
from .pivot_fetch import sweep_command, sweep_pull
from .three_recipient import move
from .timed_keeper import TimedGoalCover


class GoalLaneCover(TimedGoalCover):
    def commands(self, world, presser, needed, preparing=False):
        commands = super().commands(world, presser, needed, preparing)
        if not commands:
            return commands
        keeper_key = next((key for key in commands if key.id == 0), None)
        if keeper_key is None or not possession(world, keeper_key, CFG):
            return commands
        keeper = world.robot(keeper_key)
        rival = opposing_holder(world, presser.key.team)
        if rival is None:
            return commands
        sign = world.direction(presser.key.team)
        # Prefer the carrier's flank that leads away from our own goal.
        side = max((-1, 1), key=lambda side: sign * sweep_pull(rival, side)[0])
        if sign * sweep_pull(rival, side)[0] <= 0:
            return commands
        commands[keeper_key] = sweep_command(world, keeper, rival, side, keeper.yaw)
        if presser.key in commands and sign * presser.x >= -PENALTY_FRONT + 0.04:
            # Stay fieldward of the breakout and keep the second roller off
            # until ordinary possession or recovery selects the next action.
            ball = world.ball
            target = legal_off_ball_target(
                (sign * max(-PENALTY_FRONT + 0.25, sign * ball.x + 0.9), ball.y)
            )
            face = math.atan2(ball.y - presser.y, ball.x - presser.x)
            commands[presser.key] = replace(move(presser, *target, face, speed=0.90), dribbler=0)
        return commands
