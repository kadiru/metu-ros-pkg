# RoboTurk Match Viewer

![Match viewer showing Blue 3's reception task, motion gauges, and role timeline](preview.png)

## Watch a live match

From the RoboTurk stack root, use the project's Python environment and the Webots and AutoRef setup in the [project README](../../README.md#watch-the-official-referee-control-a-5v5-match).

```sh
python scripts/role_viewer.py --play media/match-viewer/match-spec.json runs/my-live-match
```

Open [the local viewer](http://127.0.0.1:8765/) as soon as the command prints the address. The match has two 30-second halves. Use a fresh directory for each run. Use `--port` to pick another port. Set `"realtime": true` in the match spec to also show the native Webots window.

- Select a robot on the field or in a lineup. Its current and previous roles appear side by side with its task, target, and assignment reason. Use the team and robot buttons to change what appears on the field. **Solo selected** shows the selected robot and the ball. The teams continue playing with their full lineups.
- The translation dial shows commanded velocity in the robot frame, with forward at the top and the drive speed limit at the outer ring. The rotation bar shows left and right turns around zero and marks the controller's turn limits. The gauges show requested motion.
- Click a colored segment in the **30-second role timeline** or an event in the **60-second match timeline** to review that moment. Use the timeline arrows to scroll through events and **Go live** to catch up. Score, game clock, and referee commands update with play.

The viewer listens on loopback. It saves an offline replay as `runs/my-live-match/viewer.html` at full time. Press Ctrl-C in the terminal when you finish watching.

## Play a recording

[Download the standalone viewer](https://raw.githubusercontent.com/kadiru/metu-ros-pkg/codex/roboturk-ros2/ros2/roboturk/media/match-viewer/index.html) and open it in a browser. The example contains a two-half Webots match under the SSL Game Controller. Yellow won 1–0.

```sh
python scripts/role_viewer.py runs/my-live-match runs/my-live-match/viewer.html
```

Older match folders with `roles.json.gz` and `positions.npz` also export. Detailed assignment reasons and motion commands require a role audit. An audited assignment shows the recorded rule for a reserved robot, or target travel distance, a 0.42 cost for switching an existing role, and the winning lineup's combined cost across the four field robots.
