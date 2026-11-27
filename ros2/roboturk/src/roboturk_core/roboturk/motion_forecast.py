"""Causal robot-position baselines for saved 5v5 matches."""

from dataclasses import dataclass
import math

import numpy as np


HORIZON_S = 1.0
WARMUP_S = 0.20
MEASUREMENT_STD_M = 0.005
ACCELERATION_NOISE = 1.0  # white acceleration spectral density, m²/s³
OPEN_PLAY = frozenset(("Attack", "Defend", "Recover"))


def ticks_for(seconds, dt):
    return math.ceil(seconds / dt - 1e-9)


def open_play_segments(frames, dt):
    """Give each contiguous open-play interval an ID and mark other frames -1."""
    segments = np.full(len(frames), -1, dtype=int)
    marker = None
    previous_time = None
    segment = -1
    for index, frame in enumerate(frames):
        timestamp = frame.get("clock", index * dt)
        if (
            frame.get("mode", "running") != "running"
            or frame.get("restricted_to")
            or len(frame["phases"]) != 2
            or not OPEN_PLAY.issuperset(frame["phases"])
        ):
            marker = None
            continue
        key = (
            frame.get("restart"), frame.get("attack_sign"), frame.get("stage"),
            tuple(frame["score"]),
        )
        if key != marker or not math.isclose(timestamp - previous_time, dt, abs_tol=1e-3):
            segment += 1
        segments[index] = segment
        marker, previous_time = key, timestamp
    return segments


@dataclass(frozen=True)
class Forecasts:
    # The last axis contains x, y, vx, vy in field coordinates.
    current: np.ndarray
    kalman: np.ndarray
    valid: np.ndarray

    def viewer_data(self):
        # The viewer already has the observed starting position of the current model.
        packed = np.concatenate((self.current[..., 2:], self.kalman), axis=-1)
        return {
            "horizon": HORIZON_S,
            "states": [
                np.round(packed[tick].ravel(), 5).tolist() if valid else None
                for tick, valid in enumerate(self.valid)
            ],
        }


def replay_forecasts(positions, dt, segments):
    """Filter XY samples in order, using the same measurement stream for both arms."""
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("Expected a positive recording step")
    if positions.ndim != 3 or positions.shape[-1] != 2 or len(positions) != len(segments):
        raise ValueError("Expected frame × robot × XY positions with a segment per frame")
    if not np.isfinite(positions).all():
        raise ValueError("Robot positions must be finite")

    transition = np.array([[1.0, dt], [0.0, 1.0]])
    process = ACCELERATION_NOISE * np.array([[dt**3 / 3, dt**2 / 2], [dt**2 / 2, dt]])
    variance = MEASUREMENT_STD_M**2
    current = np.zeros((*positions.shape[:2], 4))
    kalman = np.zeros_like(current)
    current[..., :2] = positions
    horizon, warmup = ticks_for(HORIZON_S, dt), ticks_for(WARMUP_S, dt)
    valid = np.zeros(len(positions), dtype=bool)
    previous = -1
    for tick, (sample, segment) in enumerate(zip(positions, segments, strict=True)):
        if segment < 0:
            previous = -1
            continue
        if segment != previous:
            state = np.stack((sample, np.zeros_like(sample)), axis=-1)
            covariance = np.diag([variance, 1.0])
            start = tick
        else:
            current[tick, :, 2:] = (sample - positions[tick - 1]) / dt
            state = state @ transition.T
            covariance = transition @ covariance @ transition.T + process
            gain = covariance[:, 0] / (covariance[0, 0] + variance)
            state += (sample - state[..., 0])[..., None] * gain
            correction = np.eye(2)
            correction[:, 0] -= gain
            covariance = correction @ covariance @ correction.T + variance * np.outer(gain, gain)
        kalman[tick] = np.concatenate((state[..., 0], state[..., 1]), axis=-1)
        previous = segment
        valid[tick] = (
            tick - start >= warmup
            and tick + horizon < len(segments)
            and segments[tick + horizon] == segment
        )
    return Forecasts(current, kalman, valid)


def recorded_positions(positions, origins, seconds, dt):
    """Interpolate observations when a requested horizon falls between frames."""
    offset = np.asarray(seconds) / dt
    left = np.floor(offset + 1e-9).astype(int)
    right = np.ceil(offset - 1e-9).astype(int)
    weight = np.clip(offset - left, 0, 1)[None, :, None, None]
    return (
        (1 - weight) * positions[origins[:, None] + left]
        + weight * positions[origins[:, None] + right]
    )


def position_errors(forecasts, positions, seconds, dt):
    """Return matched errors indexed by origin, horizon, robot and model."""
    origins = np.flatnonzero(forecasts.valid)
    states = np.stack((forecasts.current[origins], forecasts.kalman[origins]), axis=-2)
    predicted = states[:, None, :, :, :2] + seconds[None, :, None, None, None] * (
        states[:, None, :, :, 2:]
    )
    actual = recorded_positions(positions, origins, seconds, dt)
    return np.linalg.norm(predicted - actual[..., None, :], axis=-1)
