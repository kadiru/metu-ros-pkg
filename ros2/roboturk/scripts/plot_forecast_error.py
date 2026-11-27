"""Score fitted robot predictors at common horizons on clean Webots recordings."""

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import numpy as np

from benchmark_motion_forecast import match_argument
from role_viewer import read_recording
from roboturk.motion_forecast import recorded_positions, replay_forecasts, open_play_segments
from roboturk.team_forecast import HORIZONS, TEAMS, TeamModel, scene_features, team_features


LEAD_MS = np.arange(40, 1001, 40)
LEAD_SECONDS = LEAD_MS / 1000
GROUPS = {
    "teammates": "Teammates",
    "opponents": "Opponents",
    "timed_teammates": "Teammates with timed routes",
}
METHODS = {
    "current": ("Straight-line", "#18283A", "-"),
    "kalman": ("Kalman", "#687584", (0, (3, 2))),
    "context": ("Context only", "#395CAF", "-."),
    "context_and_plans": ("Context + plans", "#087E70", "-"),
    "commands_and_targets": ("Context + plans, route samples masked", "#A45324", (0, (6, 3))),
}


def interpolate_forecast(prediction, origin):
    """Evaluate the path drawn in Hoca between the model's 200 ms samples."""
    knots = np.r_[0, HORIZONS]
    path = np.concatenate((origin[:, :, None, :], prediction), axis=2)
    right = np.searchsorted(knots, LEAD_SECONDS, side="left")
    left = right - 1
    weight = (LEAD_SECONDS - knots[left]) / (knots[right] - knots[left])
    return path[:, :, left] * (1 - weight[:, None]) + path[:, :, right] * weight[:, None]


def score(matches, context_model, plan_model):
    sums = defaultdict(lambda: np.zeros(len(LEAD_MS)))
    counts = defaultdict(int)

    def add(group, method, errors):
        sums[group, method] += errors.sum(axis=0)
        counts[group, method] += len(errors)

    for name, folder in matches:
        log, samples = read_recording(folder)
        if len(log["keys"]) != 10:
            raise ValueError(f"Expected a 5v5 recording for {name}")
        scene = scene_features(log, samples)
        if not len(scene.origins):
            raise ValueError(f"No full forecasting windows in {name}")
        filters = replay_forecasts(
            scene.source_xy, scene.dt, open_play_segments(log["frames"], scene.dt)
        )
        np.testing.assert_array_equal(scene.origins, np.flatnonzero(filters.valid))
        truth = recorded_positions(
            scene.source_xy, scene.origins, LEAD_SECONDS, scene.dt
        ).swapaxes(1, 2)
        shared = {}
        for method, states in (("current", filters.current), ("kalman", filters.kalman)):
            states = states[scene.origins]
            points = states[:, :, None, :2] + states[:, :, None, 2:] * LEAD_SECONDS[:, None]
            shared[method] = np.linalg.norm(points - truth, axis=-1)
        context = context_model.predict(scene.context, scene)
        shared["context"] = np.linalg.norm(
            interpolate_forecast(context, scene.positions) - truth, axis=-1
        )
        for team in TEAMS:
            teammates = np.array([key[0] == team for key in log["keys"]])
            features = team_features(log, samples, scene, team)
            timed = features[..., 1] == 1

            def plan_errors(plan):
                prediction = plan_model.predict(np.concatenate((scene.context, plan), axis=-1), scene)
                return np.linalg.norm(
                    interpolate_forecast(prediction, scene.positions) - truth, axis=-1
                )

            errors = shared | {"context_and_plans": plan_errors(features)}
            for method, values in errors.items():
                add("teammates", method, values[:, teammates].reshape(-1, len(LEAD_MS)))
                add("opponents", method, values[:, ~teammates].reshape(-1, len(LEAD_MS)))
                if timed.any():
                    add("timed_teammates", method, values[timed])
            if timed.any():
                masked = team_features(log, samples, scene, team, use_timed_paths=False)
                add("timed_teammates", "commands_and_targets", plan_errors(masked)[timed])
    return {key: (total / counts[key], counts[key]) for key, total in sums.items()}


def output_csv(curves, path):
    with path.open("w", newline="") as file:
        writer = csv.writer(file, lineterminator="\n")
        writer.writerow(("group", "method", "lead_ms", "mean_xy_error_m", "cases"))
        for group in GROUPS:
            for method in METHODS:
                if (group, method) in curves:
                    errors, count = curves[group, method]
                    writer.writerows(
                        (group, method, int(ms), f"{error:.8f}", count)
                        for ms, error in zip(LEAD_MS, errors, strict=True)
                    )


def plot(curves, matches, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.ticker import MultipleLocator

    ink = "#18283A"
    muted = "#506174"
    background = "#F8FAFC"
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "text.color": ink,
        "axes.labelcolor": ink,
        "xtick.color": muted,
        "ytick.color": muted,
    })
    groups = [name for name in GROUPS if (name, "current") in curves]
    upper = np.ceil((max(errors.max() for errors, _ in curves.values()) + .02) * 10) / 10
    figure, axes = plt.subplots(
        ncols=len(groups), sharey=True, squeeze=False, figsize=(4.4 * len(groups), 5.2),
        facecolor=background,
    )
    figure.subplots_adjust(left=.074, right=.977, bottom=.255, top=.747, wspace=.18)
    figure.text(.075, .927, "Robot forecast error by lead time", size=21, weight="bold")
    figure.text(
        .076, .863,
        f"{matches} held-out Webots recordings  /  Clean simulator positions  /  Lower is better",
        size=11, color=muted,
    )
    methods_used = []
    for group, axis in zip(groups, axes.flat, strict=True):
        axis.set_facecolor(background)
        axis.set_title(GROUPS[group], loc="left", fontweight="bold", pad=23)
        axis.text(
            0, 1.019, f"{curves[group, 'current'][1]:,} matched robot forecasts",
            transform=axis.transAxes, fontsize=9, color=muted,
        )
        for method, (label, color, style) in METHODS.items():
            if (group, method) not in curves:
                continue
            if method not in methods_used:
                methods_used.append(method)
            values = curves[group, method][0]
            axis.plot(LEAD_MS, values, color=color, linestyle=style, linewidth=2.1, label=label)
            if method in ("context", "context_and_plans", "commands_and_targets"):
                knots = np.isin(LEAD_MS, np.rint(HORIZONS * 1000))
                axis.scatter(LEAD_MS[knots], values[knots], color=color, s=11, zorder=3)
        axis.set(xlim=(0, 1020), ylim=(0, upper), xlabel="Forecast horizon ΔT (ms)")
        axis.xaxis.set_major_locator(MultipleLocator(200))
        axis.yaxis.set_major_locator(MultipleLocator(.1))
        axis.grid(axis="y", color="#DCE3E9", linewidth=.8)
        axis.set_axisbelow(True)
        axis.tick_params(length=0, pad=6)
        axis.spines[["top", "right", "left"]].set_visible(False)
        axis.spines["bottom"].set_color("#BAC8D4")
    axes[0, 0].set_ylabel("Mean XY position error (m)", labelpad=9)
    figure.legend(
        handles=[
            Line2D([], [], color=METHODS[method][1], linestyle=METHODS[method][2],
                   linewidth=2.2, label=METHODS[method][0])
            for method in methods_used
        ],
        loc="lower left", bbox_to_anchor=(.071, .095), ncols=len(methods_used), frameon=False,
        fontsize=9.5, handlelength=3, columnspacing=1.4,
    )
    figure.text(
        .075, .061,
        "Scored every 40 ms. Dots mark the learned models’ native 200 ms outputs. "
        "Intermediate positions are interpolated.",
        size=9, color=muted,
    )
    figure.savefig(path, dpi=150, facecolor=background)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("matches", nargs="+", type=match_argument)
    parser.add_argument("--context-model", type=Path, required=True)
    parser.add_argument("--plan-model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    curves = score(args.matches, TeamModel.load(args.context_model), TeamModel.load(args.plan_model))
    args.output.mkdir(parents=True, exist_ok=True)
    output_csv(curves, args.output / "forecast-error-curves.csv")
    plot(curves, len(args.matches), args.output / "forecast-error-curves.png")


if __name__ == "__main__":
    main()
