"""Prioritize lateral goalkeeper arrival and clear the crossing lane under time pressure."""

from dataclasses import replace
import math

from .carrier_motor import MOTOR, track_roller
from .five_roles import CFG, PENALTY_FRONT, limit
from .goal_cover import GOAL_CHANNEL_HALF, SAVE_SPEED, GoalCover, intercept_ball, opposing_holder
from .model import possession
from .three_recipient import move

HANDOFF_WINDOW = 3 * CFG.dt


def lane_arrival(displacement, speed):
    """Earliest lateral arrival with the existing drive speed and acceleration caps."""
    remaining = max(0.0, abs(displacement) - 0.035)
    if remaining == 0:
        return 0.0
    forward = limit(math.copysign(1.0, displacement) * speed, SAVE_SPEED)
    accel = CFG.max_acceleration
    ramp_time = (SAVE_SPEED - forward) / accel
    ramp_dist = (forward + SAVE_SPEED) * ramp_time / 2
    if remaining > ramp_dist:
        return ramp_time + (remaining - ramp_dist) / SAVE_SPEED
    return (math.sqrt(forward**2 + 2 * accel * remaining) - forward) / accel


def crossing(world, team):
    """Estimate the deadline if the observed goalward motion continues."""
    ball = world.ball
    sign = world.direction(team)
    distance = CFG.ball_exit_x + sign * ball.x
    toward_goal = -sign * ball.vx
    if not 0 <= distance < 0.40 or toward_goal < 0.06:
        return None
    deadline = distance / toward_goal
    y = ball.y + min(deadline, 0.36) * ball.vy
    if abs(y) > CFG.goal_width / 2 + CFG.ball_radius:
        return None
    return deadline, limit(y, GOAL_CHANNEL_HALF - 0.018)


class TimedGoalCover(GoalCover):
    def commands(self, world, presser, needed, preparing=False):
        team = presser.key.team
        risk = crossing(world, team)
        keeper = next(
            (r for r in world.robots if r.key.team == team and r.key.id == 0 and r.visible), None
        )
        ready = False
        if risk is not None and keeper is not None and preparing:
            deadline, lane = risk
            eta = lane_arrival(lane - keeper.y, keeper.vy)
            # Arm the keeper during a safe tackle. Yield before the crossing
            # only when the keeper can actually arrive in time.
            ready = eta + CFG.dt <= deadline <= eta + HANDOFF_WINDOW
        active = super().commands(world, presser, needed or ready, preparing)
        if not active:
            return active

        sign = world.direction(team)
        ball = world.ball
        if sign * ball.x > -4.12:
            return active
        target, _ = intercept_ball(world, keeper)
        lane = risk[1] if risk is not None else limit(target[1], GOAL_CHANNEL_HALF - 0.018)
        lateral = abs(lane - keeper.y)
        # Approach the lane on the goal side of the yielding presser. Hold an
        # intercept rail in front of the line until the lateral move completes.
        if lateral > 0.13:
            depth = 4.49
        else:
            depth = min(4.54, max(4.38, -sign * ball.x + 0.045))
        rival = opposing_holder(world, team)
        if presser.key in active and sign * presser.x < -PENALTY_FRONT + 0.05:
            # Release the keeper's approach lane promptly. Leave along the
            # length of the field before changing sides to screen a pass.
            face = math.atan2(ball.y - presser.y, ball.x - presser.x)
            clear_x = sign * (-PENALTY_FRONT + 0.10)
            active[presser.key] = replace(
                move(presser, clear_x, presser.y, face, speed=1.15), dribbler=0
            )
        # Meet the ball with the roller, even when arriving from its flank.
        # Aim in front of the line so the roller spring cannot drag it across.
        aim_x = sign * max(-CFG.length / 2 + 0.012, sign * (ball.x + 0.08 * ball.vx))
        facing = math.atan2(lane - keeper.y, aim_x - keeper.x)
        if possession(world, keeper.key, CFG):
            # Once the roller reaches the contested ball, start the fieldward
            # pull without waiting for the other roller to release it.
            active[keeper.key] = track_roller(world, keeper, (sign * 0.64, 0.0), 0.0, MOTOR)
        elif rival is None and lateral <= 0.09 and sign * ball.x > -4.40:
            # With the lane already covered, move the roller onto a loose ball.
            active[keeper.key] = intercept_ball(world, keeper)[1]
        else:
            save = move(keeper, -sign * depth, lane, facing, speed=SAVE_SPEED)
            mouth_x = keeper.x + CFG.possession_x * math.cos(keeper.yaw)
            active[keeper.key] = (
                save if sign * mouth_x > -CFG.length / 2 else replace(save, dribbler=0)
            )
        return active
