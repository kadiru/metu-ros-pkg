"""A shared candidate scorer trained on outcomes of two-sided passing decisions."""

from dataclasses import dataclass
import math
import numpy as np

from .model import body_vector, Command, wrap


def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def candidates(world, carrier):
    """Sorting identifies a teammate for logging only. Weights are shared by both."""
    mates = sorted(
        (r for r in world.robots if r.key.team == carrier.key.team and r.key != carrier.key),
        key=lambda r: r.key.id,
    )
    if len(mates) != 2:
        raise ValueError("Recipient selection requires two teammates")
    return mates


def features(world, carrier, receiver):
    dx, dy = receiver.x - world.ball.x, receiver.y - world.ball.y
    length = max(0.01, math.hypot(dx, dy))
    ux, uy = dx / length, dy / length
    opponents = [r for r in world.robots if r.key.team != carrier.key.team]
    gaps = []
    for r in opponents:
        x, y = r.x - world.ball.x, r.y - world.ball.y
        along = min(length, max(0, x * ux + y * uy))
        gaps.append(math.hypot(x - along * ux, y - along * uy))
    return np.array(
        [
            min(1, min(gaps) / 1.2),
            min(1, min(distance(r, receiver) for r in opponents) / 2),
            dx * world.direction(carrier.key.team) / 4,
            length / 4,
            math.cos(math.atan2(dy, dx) - carrier.yaw),
            math.cos(math.atan2(-dy, -dx) - receiver.yaw),
        ]
    )


@dataclass
class Choice:
    receiver: int
    slot: int
    features: list
    probability: float


def choose(world, carrier, weights, rng=None):
    mates = candidates(world, carrier)
    rows = np.stack([features(world, carrier, mate) for mate in mates])
    diff = np.clip((rows[1] - rows[0]) @ weights, -30, 30)
    p = float(1 / (1 + np.exp(-diff)))
    slot = int(p > 0.5) if rng is None else int(rng.random() < p)
    return Choice(mates[slot].key.id, slot, rows.tolist(), p)


def train_update(weights, decisions, rate=0.4):
    """On-policy score-function gradient for actual pass return with a constant baseline."""
    if not decisions:
        return weights.copy()
    gradients = []
    for event in decisions:
        first, second = np.asarray(event["features"])
        gradients.append(
            (event["slot"] - event["probability"]) * (second - first) * event["reward"]
        )
    return weights + rate * np.mean(gradients, axis=0)


def move(robot, x, y, angle, speed=1.1):
    dx, dy = x - robot.x, y - robot.y
    scale = min(3.8, speed / max(0.001, math.hypot(dx, dy)))
    vx, vy = body_vector(dx * scale, dy * scale, robot.yaw)
    return Command(vx, vy, max(-5, min(5, 5 * wrap(angle - robot.yaw))), 1)
