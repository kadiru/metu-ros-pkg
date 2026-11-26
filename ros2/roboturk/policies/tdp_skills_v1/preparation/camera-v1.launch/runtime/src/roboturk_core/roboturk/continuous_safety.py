"""Optional geometric velocity safety layer, independent of tactical decisions.

Projects all four requested translations jointly onto robot-separation and field
constraints. Rotation, dribbler and kick commands are untouched. This is an
analytic safety controller, not a learned skill, and must be reported as such.
It is not enabled by importing this module.
"""

from dataclasses import replace
import numpy as np
from .model import Referee, body_vector, world_vector

CONTRACT = "roboturk.velocity_safety.v1"
SEPARATION = 0.205
FIELD_BOUNDS = (4.35, 2.85)
GAIN = 4.0
SPEED = 1.2


def constraints(poses):
    n = len(poses)
    rows, limits = [], []
    for i, pose in enumerate(poses):
        for axis, bound in enumerate(FIELD_BOUNDS):
            for sign in (-1, 1):
                row = np.zeros(2 * n)
                row[2 * i + axis] = sign
                rows.append(row)
                limits.append(GAIN * (bound - sign * pose[axis]))
        for j in range(i):
            delta = np.asarray(pose[:2]) - np.asarray(poses[j][:2])
            distance = float(np.linalg.norm(delta))
            if distance < 1e-8:
                raise ValueError("Coincident robots: no geometric separation direction")
            unit = delta / distance
            row = np.zeros(2 * n)
            row[2 * i : 2 * i + 2] = -unit
            row[2 * j : 2 * j + 2] = unit
            rows.append(row)
            limits.append(GAIN * (distance**2 - SEPARATION**2) / (2 * distance))
    return np.asarray(rows), np.asarray(limits)


def project(poses, requested):
    raw = np.asarray(requested, dtype=np.float64).reshape(-1)
    a, b = constraints(poses)

    def violation(x):
        return max(
            float(np.max(a @ x - b)),
            float(np.max(np.linalg.norm(x.reshape(-1, 2), axis=1) - SPEED)),
        )

    if violation(raw) <= 1e-9:
        return raw.reshape(-1, 2).copy(), 0
    # Dykstra's projection onto the intersection of halfspaces and speed disks.
    # Corrections preserve the least-squares optimum rather than implementing an
    # ordered tactical repulsion heuristic. Failure to converge is explicit.
    x = raw.copy()
    corrections = np.zeros_like(a)
    disk_corrections = np.zeros((len(poses), 2))
    squared_norms = (a * a).sum(1)
    for iteration in range(512):
        previous = x.copy()
        for i, normal in enumerate(a):
            candidate = x + corrections[i]
            offset = max(0.0, float(normal @ candidate - b[i])) / squared_norms[i]
            x = candidate - offset * normal
            corrections[i] = candidate - x
        for i in range(len(poses)):
            candidate = x[2 * i : 2 * i + 2] + disk_corrections[i]
            projected = candidate / max(1.0, float(np.linalg.norm(candidate)) / SPEED)
            disk_corrections[i] = candidate - projected
            x[2 * i : 2 * i + 2] = projected
        if np.max(np.abs(x - previous)) < 1e-10 and violation(x) <= 1e-8:
            return x.reshape(-1, 2), iteration + 1
    raise RuntimeError("Velocity safety projection did not converge")


def filter_commands(world, commands):
    if world.referee != Referee.RUNNING:
        return commands, None
    keys = sorted(commands)
    poses = [[world.robot(k).x, world.robot(k).y, world.robot(k).yaw] for k in keys]
    raw = np.asarray(
        [world_vector(commands[k].vx, commands[k].vy, world.robot(k).yaw) for k in keys]
    )
    applied, iterations = project(poses, raw)
    corrections = np.linalg.norm(applied - raw, axis=1)
    if np.max(corrections) < 1e-8:
        return commands, None
    output = dict(commands)
    for k, velocity in zip(keys, applied, strict=True):
        vx, vy = body_vector(*velocity, world.robot(k).yaw)
        output[k] = replace(commands[k], vx=float(vx), vy=float(vy))
    return output, {
        "robots": [f"{k.team}{k.id}" for k in keys],
        "poses": poses,
        "requested": raw.tolist(),
        "applied": applied.tolist(),
        "corrections": corrections.tolist(),
        "solver_iterations": iterations,
    }
