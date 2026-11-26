"""Optional safety layer is separate from learned behavior and unchanged by colors."""

from dataclasses import replace
import math
import numpy as np

from roboturk.continuous_safety import project, constraints, filter_commands, SPEED
from roboturk.continuous import KEYS
from roboturk.model import Ball, Command, Referee, Robot, World


def test_joint_projection_preserves_feasible_motion_and_handles_boundaries():
    poses = [[-0.13, 0, 0], [0.13, 0, math.pi], [4.34, 2.84, 0], [-2, 1, 0]]
    requested = np.array([[1, 0], [-1, 0], [1, 1], [0, 0]], float)
    applied, iterations = project(poses, requested)
    a, b = constraints(poses)
    assert iterations > 0 and np.max(a @ applied.ravel() - b) < 1e-7
    assert np.max(np.linalg.norm(applied, axis=1)) <= SPEED + 1e-7
    again, iterations = project(poses, applied)
    np.testing.assert_allclose(again, applied, atol=1e-7)
    # Reversing team/robot ordering must yield the same joint optimum.
    reversed_result, _ = project(poses[::-1], requested[::-1])
    np.testing.assert_allclose(applied, reversed_result[::-1], atol=1e-7)
    rotated, _ = project([[-x, -y, yaw + math.pi] for x, y, yaw in poses], -requested)
    np.testing.assert_allclose(applied, -rotated, atol=1e-7)


def test_filter_leaves_rotation_release_and_referee_stop_unchanged():
    world = World(
        0, (Robot(KEYS[0], -0.13, 0), Robot(KEYS[2], 0.13, 0, math.pi)), Ball(0, 1), Referee.RUNNING
    )
    commands = {k: Command(vx=1, omega=0.5, dribbler=0.7, kick=2.2) for k in (KEYS[0], KEYS[2])}
    applied, record = filter_commands(world, commands)
    assert record and all(c > 0 for c in record["corrections"])
    for key in commands:
        for attr in ("omega", "dribbler", "kick"):
            assert getattr(applied[key], attr) == getattr(commands[key], attr)
    stopped = replace(world, referee=Referee.HALT)
    assert filter_commands(stopped, commands) == (commands, None)
