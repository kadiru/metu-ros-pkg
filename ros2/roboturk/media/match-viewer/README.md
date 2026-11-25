# RoboTurk Match Viewer

![Practice match showing the shared motion sliders in the top bar](preview.png)

## Start a live match

Run this from the RoboTurk stack root in the project's Python environment. Use the Webots and AutoRef setup in the [project README](../../README.md#watch-the-official-referee-control-a-5v5-match).

```sh
python scripts/role_viewer.py --setup media/match-viewer/match-spec.json runs/viewer-sessions
```

Open [the local setup page](http://127.0.0.1:8765/). Both modes run an officiated 5v5 Webots match with two 30-second halves. The session saves a replay under its `run/viewer.html` path. Use `--port` to pick a different port and Ctrl-C to stop the service. Set `"realtime": true` in the preset to also show the native Webots window.

| Mode | Match setup |
| --- | --- |
| Practice | Choose each team's strategy and passing preference. The four top-bar sliders control all ten robots. |
| Game | Choose your team color. Both teams use the Balanced playbook and fixed limits. |

Set **Speed**, **Turn**, **Accel**, and **Kick** in the top bar before kickoff or while following a live Practice match. Slider changes take effect together for both teams on the next control tick that reads them. The motion gauges and replay use the limits recorded for the selected moment. Game and replay controls are read-only.

| Setting | Practice range | Game |
| --- | ---: | ---: |
| Planar top speed | 0.5–5 m/s | 3 m/s |
| Top rotation | 1–16 rad/s | ±8 rad/s |
| Acceleration and braking | 0.5–10 m/s² | 5 m/s² |
| Flat kick speed | 0.5–10 m/s | 6.5 m/s |

Practice starts with the Game limits. The open-play planner uses the selected speed and acceleration for travel and braking. Clear, lined-up shots request the selected kick speed. Passes keep their reception-tuned speeds, and restarts, close challenges, and ball control still use slower approaches. The SSL rulebook sets the 6.5 m/s ball-speed cap. The 5 m/s² drive setting comes from RoboTurk's motion model. [SSL ball-speed rule](https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_speed) · [RoboTurk drive and kicker tests](../ssl-physics/README.md#drive-and-kicker)

Select a robot on the field or a lineup to inspect its assignment. The inspector shows the strategy, play and current step, the current and previous roles, and the requested motion. New recordings show measured Webots speed below the translation command and kick speed on shot events. The colored **10-second role timeline** and **60-second match-event timeline** let you jump to earlier moments. **Go live** catches up. Team and player filters change what the viewer draws. They leave the active robots on the simulated field.

## Initial 5v5 playbooks

| Strategy | On-ball attack play | Effect |
| --- | --- | --- |
| Balanced | Build the attack | Uses the existing two wing outlets, anchor, pass triggers, and shot range. |
| Wide build-up | Work the wings | Moves outlets farther apart and makes the carrier pass earlier. |
| Direct attack | Drive to goal | Pushes outlets upfield, favors carrying longer, and considers open shots from farther away. |

**Passing preference** adjusts when an eligible receiver is used relative to the selected playbook. The shot-opening requirement, receiver eligibility checks, referee restrictions, and motion planner still apply.

The CMDragons system selects team plays using applicability conditions and fills the play's robot roles dynamically. Its Skills, Tactics and Plays paper calls the coordinated actions *steps* in a sequence of role tactics. RoboTurk uses possession and referee state to select a play, and shows the observed step such as lining up a pass or receiving it. The initial strategy presets are rule-based and use the existing skill controllers. Full synchronized tactic sequences are a future extension. [Original STP paper](https://www.cs.cmu.edu/~mmv/papers/05STP-IEEE.pdf) · [CMDragons 2016 team description](https://ssl.robocup.org/wp-content/uploads/2019/01/2016_ETDP_CMDragons.pdf)

## Play a recording

[Download the standalone match viewer](https://raw.githubusercontent.com/kadiru/metu-ros-pkg/codex/roboturk-ros2/ros2/roboturk/media/match-viewer/index.html) and open it in a browser. Scrub across 0:08 in the Practice example to see the shared limits change.

```sh
python scripts/role_viewer.py runs/viewer-sessions/SESSION/run runs/viewer-sessions/SESSION/run/viewer.html
```

Older recordings with `roles.json.gz` and `positions.npz` also export. For older asymmetric limits, the top bar shows Blue / Yellow values. Detailed assignment reasons and motion commands require a role audit. The team-plan visualization uses play data recorded by the current controller.
