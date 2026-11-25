# RoboTurk Match Viewer

[Download the standalone viewer](https://raw.githubusercontent.com/kadiru/metu-ros-pkg/codex/roboturk-ros2/ros2/roboturk/media/match-viewer/index.html) and open it in a browser. The match is embedded in the file and plays offline. Yellow won this Webots game 1–0 under the SSL Game Controller, with two 30-second halves.

![Match viewer showing the field, teams, referee state, and selected robot audit](preview.png)

- Select any robot on the field or in the lineup. The inspector shows its current task, previous distinct role, recent role changes, and the command issued at that moment. The dashed line marks its assigned target.
- Use the timeline or a match event to jump to a moment. The score, referee phase, ball state, and team lineups follow the recording.
- The assignment reason comes from the controller trace. For a cost-based assignment, it shows the robot's distance to the target, the 0.42 cost added for switching an existing role, and the winning lineup's combined cost across the four field robots. A fixed keeper or a robot reserved for an active pass or possession shows the recorded rule.

## Record another match

Run these commands from the RoboTurk stack root with the Webots and AutoRef setup described in the project README. The preset enables the role audit.

```sh
python scripts/autoref/run_placement.py media/match-viewer/match-spec.json runs/new-role-match
python scripts/role_viewer.py runs/new-role-match runs/new-role-match/viewer.html
```

Older match folders with `roles.json.gz` and `positions.npz` also export. They show role history and general role descriptions. Detailed per-tick assignment reasons and commands require the role audit.
