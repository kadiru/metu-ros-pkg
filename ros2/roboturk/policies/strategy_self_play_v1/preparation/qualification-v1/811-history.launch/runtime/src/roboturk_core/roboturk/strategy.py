"""Strategy selects a play; a play assigns roles; roles issue replaceable skill intents."""

import math

from .model import Config, Intent, Play, Referee, Skill, possession


def clear_lane(start, end, opponents, margin=0.22):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length2 = dx * dx + dy * dy
    if length2 < 1e-9:
        return False
    for robot in opponents:
        t = max(0.0, min(1.0, ((robot.x - start[0]) * dx + (robot.y - start[1]) * dy) / length2))
        if math.hypot(robot.x - start[0] - t * dx, robot.y - start[1] - t * dy) < margin:
            return False
    return True


class TeamStrategy:
    def __init__(self, team="blue", cfg=Config()):
        self.team, self.cfg = team, cfg
        self.pass_pair = None
        self.pass_until = 0.0

    def reset(self):
        self.pass_pair = None
        self.pass_until = 0.0

    def plan(self, world):
        robots = sorted(
            (r for r in world.robots if r.key.team == self.team and r.visible),
            key=lambda r: r.key.id,
        )
        if world.referee != Referee.RUNNING or not world.ball.visible:
            self.reset()
            return Play("halt", {r.key: Intent(Skill.HOLD, (r.x, r.y)) for r in robots})
        if not robots:
            return Play("empty")
        sign = world.direction(self.team)
        goal = (sign * self.cfg.length / 2, 0.0)
        play = Play("attack")
        field_players = robots
        if len(robots) >= 3:
            goalie = robots[0]
            play.assignments[goalie.key] = Intent(
                Skill.MOVE,
                (
                    -goal[0] + sign * 0.25,
                    max(-self.cfg.goal_width / 2, min(self.cfg.goal_width / 2, world.ball.y)),
                ),
            )
            field_players = robots[1:]
        carrier = next((r for r in field_players if possession(world, r.key, self.cfg)), None)
        striker = carrier or min(
            field_players, key=lambda r: math.hypot(r.x - world.ball.x, r.y - world.ball.y)
        )
        for index, r in enumerate(field_players):
            play.assignments[r.key] = Intent(
                Skill.MOVE, (sign * 0.5, (-1 if index % 2 else 1) * (0.7 + 0.3 * index))
            )
        # A pass is latched so roles don't swap while the ball travels.
        if self.pass_pair:
            passer, receiver = self.pass_pair
            keys = {r.key for r in field_players}
            if (
                passer in keys
                and receiver in keys
                and world.time < self.pass_until
                and not possession(world, receiver, self.cfg)
            ):
                rr = world.robot(receiver)
                play.name = "pass_and_receive"
                play.assignments[passer] = Intent(
                    Skill.PASS if possession(world, passer, self.cfg) else Skill.HOLD,
                    (rr.x, rr.y),
                    receiver,
                )
                play.assignments[receiver] = Intent(Skill.RECEIVE, (rr.x, rr.y))
                return play
            self.pass_pair = None
        if not carrier:
            play.name = "recover_ball"
            play.assignments[striker.key] = Intent(Skill.CHASE, goal)
            return play
        opponents = [r for r in world.robots if r.key.team != self.team and r.visible]
        start = (world.ball.x, world.ball.y)
        if clear_lane(start, goal, opponents) and abs(goal[0] - carrier.x) < 3.0:
            play.name = "shoot"
            play.assignments[carrier.key] = Intent(Skill.SHOOT, goal)
        else:
            candidates = [
                r
                for r in field_players
                if r.key != carrier.key
                and sign * (r.x - carrier.x) > 0.3
                and clear_lane(start, (r.x, r.y), opponents)
            ]
            if candidates:
                receiver = max(candidates, key=lambda r: sign * r.x)
                self.pass_pair = (carrier.key, receiver.key)
                self.pass_until = world.time + 3.0
                play.name = "pass_and_receive"
                play.assignments[carrier.key] = Intent(
                    Skill.PASS, (receiver.x, receiver.y), receiver.key
                )
                play.assignments[receiver.key] = Intent(Skill.RECEIVE, (receiver.x, receiver.y))
            else:
                play.name = "advance"
                play.assignments[carrier.key] = Intent(Skill.DRIBBLE, goal)
        return play
