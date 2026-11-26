"""Pass motor invariants; native trials establish performance separately."""

import math
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
from roboturk.model import Config, RobotKey, wrap
from roboturk.continuous import Memory, KEYS
from roboturk.strategy_options import formation
from roboturk.strategy_pass import observation, pass_command


def context(w):
    memory = Memory()
    memory.reset(w)
    return SimpleNamespace(world=w, memory=memory, cfg=Config())


def test_motor_observation_is_invariant_to_whole_field_rotation():
    w, _ = formation(6103102)
    rotated = replace(
        w,
        attack_sign=-w.attack_sign,
        ball=replace(w.ball, x=-w.ball.x, y=-w.ball.y, vx=-w.ball.vx, vy=-w.ball.vy),
        robots=tuple(
            replace(r, x=-r.x, y=-r.y, vx=-r.vx, vy=-r.vy, yaw=wrap(r.yaw + math.pi))
            for r in w.robots
        ),
    )
    for key in KEYS:
        np.testing.assert_allclose(
            observation(context(w), key), observation(context(rotated), key), atol=1e-6
        )


def test_teacher_does_not_fire_before_stable_aligned_possession():
    from roboturk.model import possession

    w, _ = formation(6103102)
    m = context(w)
    key = next(k for k in KEYS if possession(w, k, m.cfg))
    assert pass_command(m, key).kick == 0
    m.memory.dwell[key] = 20
    mate = w.robot(RobotKey(key.team, 1 - key.id))
    r = w.robot(key)
    yaw = math.atan2(mate.y - w.ball.y, mate.x - w.ball.x)
    aligned = replace(
        r, yaw=yaw, x=w.ball.x - 0.115 * math.cos(yaw), y=w.ball.y - 0.115 * math.sin(yaw)
    )
    m.world = replace(w, robots=tuple(aligned if p.key == key else p for p in w.robots))
    assert pass_command(m, key).kick == 2.2
    m.world = replace(
        m.world,
        robots=tuple(
            replace(p, yaw=wrap(p.yaw + 0.1)) if p.key == key else p for p in m.world.robots
        ),
    )
    assert pass_command(m, key).kick == 0
