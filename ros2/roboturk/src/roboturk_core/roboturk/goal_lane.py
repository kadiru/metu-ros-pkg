"""Break a shared keeper grip toward the field and give the presser a clear outlet."""

import math
from dataclasses import replace

from .five_roles import CFG, PENALTY_FRONT, legal_off_ball_target, limit
from .goal_cover import opposing_holder
from .model import possession, wrap
from .pivot_fetch import sweep_command, sweep_pull
from .three_recipient import move
from .timed_keeper import TimedGoalCover


class GoalLaneCover(TimedGoalCover):
    def __init__(self, pivot=False):
        super().__init__()
        self.pivot = pivot
        self.pivots = {}
        self.pivot_samples = []

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
        yaw_target = keeper.yaw
        if self.pivot:
            previous = self.pivots.get(keeper_key)
            if (
                previous is None
                or previous[0] != rival.key
                or not 0 < world.time - previous[1] <= 1.5 * CFG.dt
            ):
                upfield = 0.0 if sign > 0 else math.pi
                yaw_target += limit(wrap(upfield - keeper.yaw), math.radians(28))
                start_yaw = keeper.yaw
            else:
                _, _, start_yaw, yaw_target = previous
            self.pivots[keeper_key] = rival.key, world.time, start_yaw, yaw_target
        command = sweep_command(world, keeper, rival, side, yaw_target)
        commands[keeper_key] = command
        if self.pivot:
            self.pivot_samples.append(
                dict(
                    time=world.time,
                    start_yaw=start_yaw,
                    yaw=keeper.yaw,
                    target_yaw=yaw_target,
                    commanded_omega=command.omega,
                )
            )
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
