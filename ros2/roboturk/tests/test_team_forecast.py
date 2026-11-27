from copy import deepcopy

import numpy as np

from roboturk.team_forecast import scene_features, team_features


def fixture():
    count = 60
    keys = [["blue", index] for index in range(5)] + [["yellow", index] for index in range(5)]
    samples = np.zeros((count, 4 + 3 * len(keys)))
    samples[:, 4::3] = np.linspace(-2, 2, len(keys))
    audit = [{"command": [0.5, 0, 0], "target": [1, 0]} for _ in keys]
    motion = [
        {"path": [[x + 0.04 * tick, 0] for tick in range(16)]}
        for x in samples[0, 4::3]
    ]
    frames = [
        {"phases": ["Attack", "Defend"], "score": [0, 0], "allocation": audit, "motion": motion}
        for _ in range(count)
    ]
    return {"keys": keys, "frames": frames, "dt": 0.04}, samples


def test_team_view_excludes_the_rivals_private_plan_but_can_use_its_observed_motion():
    log, samples = fixture()
    scene = scene_features(log, samples)
    rival_edit = deepcopy(log)
    rival_edit["frames"][0]["allocation"][5]["target"] = [9, 9]
    rival_edit["frames"][0]["motion"][5]["path"] = [[6, 7]] * 16

    np.testing.assert_array_equal(scene.context, scene_features(rival_edit, samples).context)
    blue = team_features(log, samples, scene, "blue")
    np.testing.assert_array_equal(blue, team_features(rival_edit, samples, scene, "blue"))
    assert not np.array_equal(
        team_features(log, samples, scene, "yellow"),
        team_features(rival_edit, samples, scene, "yellow"),
    )
    assert not np.any(blue[:, 5:, :12])
    assert np.any(blue[:, 5:, 12:])  # The opponent forecast can see nearby Blue plans.

    samples[:, 4 + 3 * 5] += 0.5
    assert not np.array_equal(scene.context, scene_features(log, samples).context)


def test_uses_a_timed_route_when_available_and_future_samples_do_not_enter_features():
    log, samples = fixture()
    scene = scene_features(log, samples)
    actual = team_features(log, samples, scene, "blue")
    fallback = team_features(log, samples, scene, "blue", use_timed_paths=False)
    assert actual[0, 0, 1] == 1
    assert fallback[0, 0, 1] == 0
    np.testing.assert_allclose(actual[0, 0, 4:10], [0.10, 0, 0.30, 0, 0.50, 0])

    changed = samples.copy()
    changed[12:, 4:] += 1
    other = scene_features(log, changed)
    past = scene.origins < 12
    np.testing.assert_array_equal(scene.context[past], other.context[past])
    np.testing.assert_array_equal(actual[past], team_features(log, changed, other, "blue")[past])
    assert not np.array_equal(scene.observed_future()[past], other.observed_future()[past])
