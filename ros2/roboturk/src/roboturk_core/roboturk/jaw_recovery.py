"""Learned short dribbler maneuvers using the observed ball position in both mouths."""

import math
import numpy as np

from .contest_outlet import ContestGame, sample
from .model import Ball, Command, Referee, Robot, RobotKey, World, body_vector, possession
from .three_match import CFG


def reset_pose(seed, poses, sign):
    rng = np.random.default_rng([seed, 3917])
    row = poses[int(rng.integers(len(poses)))]
    s = np.array(row["state"])
    # A rigid transformation preserves each measured contact. Renumber and
    # swap the two teams to vary which robot/side must recover.
    reflection = rng.choice([-1, 1])
    turn = rng.uniform(-math.pi, math.pi)
    c, sn = math.cos(turn), math.sin(turn)
    mat = np.array([[c, -sn], [sn, c]]) @ np.diag([1, reflection])
    positions = (s[4:].reshape(6, 3)[:, :2] - s[:2]) @ mat.T
    positions = np.clip(positions, [-3.95, -2.55], [3.95, 2.55])
    offset = rng.uniform(-0.25, 0.25, 2)
    positions += offset
    swap = int(rng.integers(2))
    robots = []
    for t, team in enumerate(("blue", "yellow")):
        ids = rng.permutation(3)
        for j, idx in enumerate(ids):
            src = 3 * (t ^ swap) + j
            yaw = turn + reflection * s[4 + src * 3 + 2]
            robots.append(Robot(RobotKey(team, int(idx)), *positions[src], yaw))
    return World(
        0, tuple(sorted(robots, key=lambda r: r.key)), Ball(*offset), Referee.RUNNING, sign
    )


def mouth(robot, ball):
    return body_vector(ball.x - robot.x, ball.y - robot.y, robot.yaw)


def context(actor, rival, ball, ticks):
    ax, ay = mouth(actor, ball)
    rx, ry = mouth(rival, ball)
    return np.clip(
        [
            1,
            (ax - 0.115) / 0.035,
            ay / 0.045,
            (rx - 0.115) / 0.035,
            ry / 0.045,
            math.sin(rival.yaw - actor.yaw),
            math.cos(rival.yaw - actor.yaw),
            min(1, ticks / 25),
        ],
        -1,
        1,
    )


# vx, lateral mouth velocity, pivot rate, use the nearer rival jaw for the sign.
MOTIONS = np.array(
    [
        [0.33, 0.31, 2.6, False],
        [0.33, -0.31, -2.6, False],
        [-0.16, 0.27, -2.8, False],
        [-0.16, -0.27, 2.8, False],
        [-0.12, 0.0, 0, False],
        [0.09, 0.0, 0, False],
        [0.06, 0.16, 2.2, True],
        [-0.09, 0.14, -1.8, True],
        [0.03, 0.06, 3.5, True],
        [0.16, 0.30, -1.6, True],
        [0.02, -0.16, -2.2, True],
        [-0.06, 0.0, 3.0, True],
    ]
)


def maneuver(index, actor, rival, ball):
    vx, vy, omega, adapt = MOTIONS[index]
    if adapt:
        ax, ay = mouth(actor, ball)
        _, ry = mouth(rival, ball)
        # Express the nearer lateral edge of the rival mouth in the actor frame.
        edge = 1 if (ry if abs(ry) > 0.003 else ay) * math.cos(rival.yaw - actor.yaw) >= 0 else -1
        omega *= edge
        # Rotate about our mouth while moving the ball sideways. Feedback
        # centers the measured ball in the dribbler during the pull.
        vx += 3.8 * (ax - 0.115)
        vy = edge * vy - 0.115 * omega + 3.8 * ay
    return Command(float(vx), float(vy), float(omega), 1, 0)


class JawGame(ContestGame):
    def __init__(self, *args, jaw_policies, poses=None, training=False, **kwargs):
        self.jaw_policies = jaw_policies
        self.poses = poses
        self.training = training
        super().__init__(*args, **kwargs)

    def reset(self, team, reason):
        super().reset(team, reason)
        if self.poses and reason == "start":
            self.world = self.backend.reset(reset_pose(self.seed, self.poses, self.sign))

    def motor_option(self, actor, rival):
        team = actor.key.team
        policy = self.jaw_policies.get(team)
        if policy is None:
            return super().motor_option(actor, rival)
        role = self.roles[team]
        x = context(actor, rival, self.world.ball, self.tick - role["start"])
        scores = np.asarray(policy["q"]) @ x
        if self.training:
            # Uniform exploration gives each motion coverage before fitting
            # the conditional returns. Frozen opponents use their learned policy.
            scores = np.zeros_like(scores) if team == self.training else scores
        k, _ = sample(scores / policy["temperature"], self.choice_rng)
        self.decisions.append(
            dict(
                team=team,
                head="jaw",
                actor=actor.key.id,
                tick=self.tick,
                action=k,
                context=x.tolist(),
            )
        )
        # The inherited controller handles support, role lifetime and kicks.
        # Its placeholder command is replaced below for active jaw maneuvers.
        role.update(action=0, jaw=k, change=self.tick + 8)

    def commands(self):
        out = super().commands()
        for team, role in self.roles.items():
            if "jaw" not in role:
                continue
            actor = self.world.robot(role["actor"])
            held = [r for r in self.world.robots if possession(self.world, r.key, CFG)]
            age = self.tick - role["last_tie"]
            if age == 0 or (age <= 3 and not (len(held) == 1 and held[0].key == actor.key)):
                rival = min(
                    (r for r in self.world.robots if r.key.team != team),
                    key=lambda r: math.hypot(r.x - self.world.ball.x, r.y - self.world.ball.y),
                )
                out[actor.key] = maneuver(role["jaw"], actor, rival, self.world.ball)
        return out


class ReturnLearner:
    """Ridge estimate of episode return per maneuver using measured jaw context."""

    def __init__(self):
        self.gram = np.repeat(np.diag([1.0, 4, 4, 4, 4, 4, 4, 4])[None, :, :], len(MOTIONS), axis=0)
        self.xy = np.zeros((len(MOTIONS), 8))

    def update(self, result):
        e = next(iter(result["contest_metrics"]["contests"]), {})
        team = result["challenger"]
        own = e.get("winner", -1) // 3 == int(team == "yellow")
        reward = (1 + 5 * ("received" in e)) if own else -1
        moves = [
            v
            for v in result["learned_actions"]
            if v["head"] == "jaw"
            and v["team"] == team
            and v["tick"] * 4 <= e.get("recovery", 100 * result["seconds"])
        ]
        for v in moves:
            i = v["action"]
            x = np.array(v["context"])
            self.gram[i] += np.outer(x, x) / len(moves)
            self.xy[i] += x * (reward + 1) / len(moves)
        return reward

    def policy(self):
        q = np.stack([np.linalg.solve(a, b) for a, b in zip(self.gram, self.xy, strict=True)])
        return dict(q=q.tolist(), temperature=0.45)
