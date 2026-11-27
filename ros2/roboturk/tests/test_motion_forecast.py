import numpy as np

from roboturk.motion_forecast import open_play_segments, position_errors, replay_forecasts


def test_constant_velocity_and_subframe_scoring():
    t = np.arange(60) * 0.04
    positions = np.stack((t, 2 * t), axis=-1)[:, None, :]
    forecasts = replay_forecasts(positions, 0.04, np.zeros(len(t), dtype=int))

    assert np.flatnonzero(forecasts.valid).tolist() == list(range(5, 35))
    errors = position_errors(forecasts, positions, np.array([0.10, 0.50, 1.0]), 0.04)
    assert errors[..., 0].max() < 1e-10
    assert errors[..., 1].max() < 1e-3


def test_future_samples_only_change_the_reference_trajectory():
    positions = np.zeros((60, 2, 2))
    segments = np.zeros(len(positions), dtype=int)
    forecasts = replay_forecasts(positions, 0.04, segments)
    changed = positions.copy()
    changed[12:, 0] = [1.0, 0.0]
    second = replay_forecasts(changed, 0.04, segments)

    np.testing.assert_array_equal(forecasts.current[11], second.current[11])
    np.testing.assert_array_equal(forecasts.kalman[11], second.kalman[11])
    errors = position_errors(second, changed, np.array([1.0]), 0.04)
    assert errors[11 - 5, 0, 0].tolist() == [1.0, 1.0]


def test_restarts_and_non_playing_frames_break_the_forecast_window():
    play = {"phases": ["Attack", "Defend"], "score": [0, 0], "restart": 0}
    frames = (
        [play] * 35
        + [dict(play, mode="stop")]
        + [dict(play, restart=1)] * 40
        + [dict(play, restart=2)] * 35
    )
    segments = open_play_segments(frames, 0.04)
    assert segments[[34, 35, 36, 75, 76]].tolist() == [0, -1, 1, 1, 2]
    positions = np.zeros((len(frames), 1, 2))
    positions[76:] = [2.0, 0.0]
    forecasts = replay_forecasts(positions, 0.04, segments)
    assert np.flatnonzero(forecasts.valid).tolist() == (
        list(range(5, 10)) + list(range(41, 51)) + list(range(81, 86))
    )
    np.testing.assert_array_equal(forecasts.kalman[76, 0], [2.0, 0.0, 0.0, 0.0])
