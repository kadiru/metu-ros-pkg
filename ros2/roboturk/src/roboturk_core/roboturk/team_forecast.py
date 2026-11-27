"""Clean-sim forecasting with a separate view of each team's private plans."""

from dataclasses import dataclass

import numpy as np

from .motion_forecast import HORIZON_S, WARMUP_S, open_play_segments, recorded_positions, ticks_for


HORIZONS = np.linspace(0.2, HORIZON_S, 5)
PLAN_TIMES = np.array([0.2, 0.6, 1.0])
MODEL_VERSION = 1
PLAN_WIDTH = 12  # available, timed path, target xy, three path xy samples, command xy
TEAMS = ("blue", "yellow")


@dataclass(frozen=True)
class Scene:
    origins: np.ndarray
    positions: np.ndarray
    direction: np.ndarray
    context: np.ndarray
    velocity: np.ndarray
    source_xy: np.ndarray
    dt: float

    def observed_future(self):
        points = recorded_positions(self.source_xy, self.origins, HORIZONS, self.dt)
        return points.swapaxes(1, 2)

    def standard_forecast(self):
        return self.positions[:, :, None, :] + self.velocity[:, :, None, :] * HORIZONS[None, :, None]


def scene_features(log, samples):
    """Features shared by both teams, built from poses, ball, score and history only."""
    frames, keys, dt = log["frames"], log["keys"], log["dt"]
    robots = samples[:, 4:].reshape(len(samples), len(keys), 3)
    xy = robots[..., :2]
    heading = np.stack((np.cos(robots[..., 2]), np.sin(robots[..., 2])), axis=-1)
    segments = open_play_segments(frames, dt)
    horizon = ticks_for(HORIZON_S, dt)
    ticks = np.arange(len(samples))
    changed = np.r_[True, segments[1:] != segments[:-1]]
    begins = np.maximum.accumulate(np.where(changed, ticks, 0))
    origins = ticks[: max(0, len(samples) - horizon)]
    origins = origins[
        (segments[origins] >= 0)
        & (segments[origins] == segments[origins + horizon])
        & (origins - begins[origins] >= ticks_for(WARMUP_S, dt))
    ]
    team_sign = np.array([1 if key[0] == "blue" else -1 for key in keys])
    sign = np.array([frames[t].get("attack_sign", log.get("sign", 1)) for t in origins])
    direction = sign[:, None] * team_sign
    angle = heading[origins]
    positions = xy[origins]
    velocity = (positions - xy[origins - 1]) / dt
    ball = samples[origins, :2]
    relative_ball = ball[:, None, :] - positions
    height = np.array([frames[t].get("ball_z") or 0 for t in origins])
    score = np.array([frames[t]["score"] for t in origins]).reshape(-1, 2)
    score_diff = (score[:, 0] - score[:, 1])[:, None] * team_sign
    ball_front_error = np.linalg.norm(relative_ball - 0.115 * angle, axis=-1)
    observed_grip = (ball_front_error < 0.045) & (height[:, None] < 0.034)
    friendly = team_sign[None, :, None] == team_sign[None, None, :]
    own_grip = np.any(observed_grip[:, None, :] & friendly, axis=-1)
    rival_grip = np.any(observed_grip[:, None, :] & ~friendly, axis=-1)
    context = [
        value * direction[..., None]
        for value in (positions, velocity, angle, relative_ball, samples[origins, None, 2:4])
    ]
    context.extend(
        np.broadcast_to(value, positions.shape[:2])[..., None]
        for value in (height[:, None], score_diff, own_grip, rival_grip)
    )
    relative = positions[:, None, :, :] - positions[:, :, None, :]
    distance = np.linalg.norm(relative, axis=-1)
    distance[:, np.arange(len(keys)), np.arange(len(keys))] = np.inf
    order = np.argsort(distance, axis=-1, kind="stable")[..., : len(keys) - 1]
    batch = np.arange(len(origins))[:, None, None]
    subject = np.arange(len(keys))[None, :, None]
    neighbors = [
        relative[batch, subject, order],
        velocity[batch, order],
        angle[batch, order],
    ]
    for lag in (0.20, 0.60, 1.0):
        past = np.maximum(origins - ticks_for(lag, dt), begins[origins])
        delta = positions - xy[past]
        context.extend(
            (
                delta * direction[..., None],
                (ball - samples[past, :2])[:, None, :] * direction[..., None],
                np.broadcast_to(((origins - past) * dt)[:, None, None], (*direction.shape, 1)),
            )
        )
        if lag < 1:
            neighbors.append(delta[batch, order])
    neighbor_features = np.concatenate(
        [value * direction[..., None, None] for value in neighbors]
        + [(team_sign[order] == team_sign[None, :, None])[..., None]],
        axis=-1,
    )
    width = (len(keys) - 1) * neighbor_features.shape[-1]
    context.append(neighbor_features.reshape(*direction.shape, width))
    return Scene(origins, positions, direction, np.concatenate(context, axis=-1), velocity, xy, dt)


def team_features(log, samples, scene, team, *, use_timed_paths=True):
    """Expose selected-team commands and paths to every prediction. Never read rival plans."""
    keys = log["keys"]
    robots = samples[:, 4:].reshape(len(samples), len(keys), 3)
    team_slots = [index for index, key in enumerate(keys) if key[0] == team]
    plans = np.zeros((*scene.direction.shape, PLAN_WIDTH), dtype=float)
    limits = log.get("limits") or {}
    acceleration = float((limits.get(team) or {}).get("max_acceleration", 5.0))
    limit_changes = {
        tick: value[team]["max_acceleration"]
        for tick, frame in enumerate(log["frames"])
        if (value := frame.get("limits")) and team in value
    }
    acceleration_by_tick = np.full(len(samples), acceleration)
    for tick, value in limit_changes.items():
        acceleration_by_tick[tick:] = value

    for row, tick in enumerate(scene.origins):
        frame = log["frames"][tick]
        audit = frame.get("allocation")
        motion = frame.get("motion")
        if not audit:
            continue
        for slot in team_slots:
            detail = audit[slot]
            command = detail.get("command")
            if command is None:
                continue
            x, y, yaw = robots[tick, slot]
            pose = np.array([x, y])
            c, s = np.cos(yaw), np.sin(yaw)
            drive = np.array([c * command[0] - s * command[1], s * command[0] + c * command[1]])
            path = motion[slot] if motion else None
            path_xy = path.get("path") if path and use_timed_paths else None
            target = (path.get("waypoint") or path.get("target")) if path else None
            target = np.asarray(target or detail.get("target") or pose.tolist()) - pose
            if path_xy:
                points = np.asarray(path_xy)
                times = np.arange(len(points)) * (2 * scene.dt)
                route = (
                    np.stack([np.interp(PLAN_TIMES, times, axis) for axis in points.T], axis=-1)
                    - pose
                )
            else:
                velocity = scene.velocity[row, slot]
                change = drive - velocity
                ramp = max(float(np.linalg.norm(change)) / acceleration_by_tick[tick], 1e-9)
                t = np.minimum(PLAN_TIMES, ramp)
                blend = t**2 / (2 * ramp) + np.maximum(PLAN_TIMES - ramp, 0)
                route = PLAN_TIMES[:, None] * velocity + blend[:, None] * change
            plans[row, slot] = np.r_[1.0, bool(path_xy), target, route.ravel(), drive]

    relative = scene.positions[:, None, team_slots, :] - scene.positions[:, :, None, :]
    distance = np.linalg.norm(relative, axis=-1)
    distance[:, team_slots, np.arange(len(team_slots))] = np.inf
    nearest = np.argsort(distance, axis=-1, kind="stable")[..., :2]
    batch = np.arange(len(scene.origins))[:, None, None]
    subject = np.arange(len(keys))[None, :, None]
    neighbor_plans = plans[batch, np.array(team_slots)[nearest]]
    own = plans.copy()
    own[..., 2:] *= scene.direction[..., None]
    neighbor_plans[..., 2:] *= scene.direction[..., None, None]
    neighbor = np.concatenate(
        (relative[batch, subject, nearest] * scene.direction[..., None, None], neighbor_plans),
        axis=-1,
    )
    return np.concatenate((own, neighbor.reshape(*scene.direction.shape, 2 * (PLAN_WIDTH + 2))), axis=-1)


@dataclass(frozen=True)
class TeamModel:
    """Portable NumPy inference for a fitted two-hidden-layer regressor."""

    mean: np.ndarray
    scale: np.ndarray
    layers: tuple

    def predict(self, features, scene):
        values = ((features - self.mean) / self.scale).astype(np.float32)
        for index, (weights, bias) in enumerate(self.layers):
            values = values @ weights.T + bias
            if index + 1 < len(self.layers):
                values = np.maximum(values, 0)
        return scene.positions[:, :, None, :] + (
            values.reshape(*scene.direction.shape, len(HORIZONS), 2)
            * scene.direction[..., None, None]
        )

    def save(self, file):
        arrays = {"version": MODEL_VERSION, "mean": self.mean, "scale": self.scale}
        for index, (weights, bias) in enumerate(self.layers):
            arrays[f"w{index}"] = weights
            arrays[f"b{index}"] = bias
        np.savez_compressed(file, **arrays)

    @classmethod
    def load(cls, file):
        with np.load(file, allow_pickle=False) as data:
            if int(data["version"]) != MODEL_VERSION:
                raise ValueError("Unsupported team forecast schema")
            layers = tuple((data[f"w{i}"], data[f"b{i}"]) for i in range(3))
            return cls(data["mean"], data["scale"], layers)


def team_predictions(log, samples, scene, model):
    return {
        team: model.predict(
            np.concatenate((scene.context, team_features(log, samples, scene, team)), axis=-1),
            scene,
        )
        for team in TEAMS
    }
