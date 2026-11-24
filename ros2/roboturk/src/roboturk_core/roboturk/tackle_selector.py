"""Choose a 5v5 tackle using the predicted direction of a loose ball."""

from dataclasses import dataclass, replace
import math

from .five_roles import CFG, legal_off_ball_target
from .goal_cover import GoalCover
from .goal_lane import GoalLaneCover
from .timed_keeper import TimedGoalCover
from .model import RobotKey, body_vector, possession, unit
from .pivot_fetch import SWEEP_RADIANS, SWEEP_SECONDS, fetch_loose, sweep_command, sweep_pull
from .three_recipient import move

DIRECT = "direct"
CONTAIN = "contain"
# Left and right refer to the ball's release side in the carrier's frame.
PIVOT_SIDES = {"pivot_left": 1, "pivot_right": -1}
ACTIONS = (DIRECT, *PIVOT_SIDES, CONTAIN)
STALL_TICKS = 12


def contain_command(world, presser):
    ball = world.ball
    sign = world.direction(presser.key.team)
    target = legal_off_ball_target((ball.x - sign * 0.48, max(-2.76, min(2.76, ball.y))))
    face = math.atan2(ball.y - presser.y, ball.x - presser.x)
    return replace(move(presser, *target, face, speed=0.70), dribbler=0)


def release_risks(ball, sign, pull):
    """Bound a possible release with measured drift and a short pull in the candidate direction."""
    px, py = unit(*pull)
    reasons = set()
    # Drift can cross a line before the roller responds. After that reaction
    # window, allow an inward pull to counter measured outward drift.
    for seconds, travel, margin in ((2 * CFG.dt, 0.0, 0.0), (0.32, 0.16, 0.025)):
        dx = seconds * ball.vx + travel * px
        dy = seconds * ball.vy + travel * py
        goal_distance = CFG.length / 2 + sign * ball.x
        toward_goal = -sign * dx
        goal_lookahead = 0.80 if travel else toward_goal
        if 0 <= goal_distance <= goal_lookahead and toward_goal > 0.015:
            goal_y = ball.y + goal_distance / toward_goal * dy
            if abs(goal_y) <= CFG.goal_width / 2 + 0.17:
                reasons.add("own_goal")
        if abs(ball.y + dy) > abs(ball.y) and abs(ball.y + dy) >= CFG.ball_exit_y - margin:
            reasons.add("touchline")
        if abs(ball.x + dx) > abs(ball.x) and abs(ball.x + dx) >= CFG.ball_exit_x - margin:
            reasons.add("endline")
    return tuple(sorted(reasons))


def mouth_error(ball, carrier):
    depth, side = body_vector(ball.x - carrier.x, ball.y - carrier.y, carrier.yaw)
    return max(abs(depth - CFG.possession_x) / 0.035, abs(side) / 0.045)


@dataclass
class Contest:
    carrier: RobotKey
    updated: float
    samples: int = 0
    initial_error: float = 0.0
    active: str = ""
    started: float = 0.0
    yaw_target: float = 0.0
    last_sweep: float | None = None
    used: tuple = ()
    last_decision: tuple = ()


class TackleSelector:
    def __init__(self, direct, keeper_cover=False, keeper_timing=False, keeper_lane=False):
        self.direct = direct
        self.contests = {}
        self.events = []
        cover = GoalLaneCover if keeper_lane else TimedGoalCover if keeper_timing else GoalCover
        self.cover = cover() if keeper_cover else None

    def command(self, world, presser, carrier, observed, has_ball):
        now, key = world.time, presser.key
        state = self.contests.get(key)
        if state is None or state.carrier != carrier.key or not 0 < now - state.updated <= 1.0:
            state = self.contests[key] = Contest(carrier.key, now)
        elif now - state.updated > 1.5 * CFG.dt:
            state.samples = 0
            state.active = ""
        state.updated = now
        ball = world.ball
        shared = has_ball and possession(world, carrier.key, CFG)
        error = mouth_error(ball, carrier)
        if shared:
            if state.samples == 0:
                state.initial_error = error
            state.samples += 1
        else:
            state.samples = 0
            state.active = ""

        sign = world.direction(key.team)
        pulls = {DIRECT: self.direct.draw_direction(presser, carrier)}
        pulls.update({action: sweep_pull(carrier, side) for action, side in PIVOT_SIDES.items()})
        vetoes = {
            action: reasons
            for action, pull in pulls.items()
            if (reasons := release_risks(ball, sign, pull))
        }
        if state.active and (state.active in vetoes or now - state.started >= SWEEP_SECONDS):
            state.active = ""
            state.samples = int(shared)
            state.initial_error = error

        # Give the direct draw time while the ball is progressing out of the carrier's mouth.
        stalled = state.samples >= STALL_TICKS and error - state.initial_error < 0.25
        if state.samples >= STALL_TICKS and not stalled:
            state.samples = 1
            state.initial_error = error
        if not state.active and shared and (stalled or DIRECT in vetoes):
            available = [
                action
                for action in PIVOT_SIDES
                if action not in vetoes and action not in state.used
            ]
            if available:

                def space(action):
                    dx, dy = unit(*pulls[action])
                    return min(
                        CFG.ball_exit_x - abs(ball.x + 0.20 * dx),
                        CFG.ball_exit_y - abs(ball.y + 0.20 * dy),
                    )

                state.active = max(available, key=space)
                state.started = now
                state.yaw_target = presser.yaw - PIVOT_SIDES[state.active] * SWEEP_RADIANS
                state.used += (state.active,)
        if state.active:
            action, reason = state.active, "unsafe_draw" if DIRECT in vetoes else "stalled_draw"
        elif DIRECT in vetoes:
            action, reason = CONTAIN, "unsafe_draw"
        else:
            action, reason = DIRECT, "draw"
        decision = action, reason, tuple(vetoes.items())
        if state.last_decision != decision:
            self.events.append(
                dict(
                    time=now,
                    presser=key,
                    carrier=carrier.key,
                    action=action,
                    reason=reason,
                    vetoes=vetoes,
                )
            )
            state.last_decision = decision
        if action in PIVOT_SIDES:
            state.last_sweep = now
            return sweep_command(world, presser, carrier, PIVOT_SIDES[action], state.yaw_target)
        if action == CONTAIN:
            return contain_command(world, presser)
        return self.direct.command(world, presser, carrier, observed, has_ball)

    def action(self, key):
        return self.contests[key].last_decision[0]

    def goal_veto(self, key):
        return any("own_goal" in risks for _, risks in self.contests[key].last_decision[2])

    def collect_loose(self, world, presser):
        state = self.contests.get(presser.key)
        return fetch_loose(world, presser, state.last_sweep) if state else None
