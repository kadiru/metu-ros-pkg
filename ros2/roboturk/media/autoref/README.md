# From a Webots shot to an official goal

Yellow scores at the left goal. TIGERs AutoRef reports a possible goal to the official SSL Game Controller, which validates it, increments Yellow's score and assigns the kickoff to Blue. At 28.20 s Blue takes the restart. At 28.48 s the ball has moved 5.06 cm and Yellow can play again. The recording ends after 1.5 more seconds of open play. [Native Webots film](webots-autoref.mp4) · [Goal and kickoff GIF](goal-kickoff.gif) · [Official events and measured restart](active.json)

| Archived 60-second match | Goals observed | Touchline exits | Goal-line exits outside the posts |
| --- | --- | ---: | ---: |
| 20300000 | Yellow, Blue, Yellow | 1 | 0 |
| 20300001 | Blue | 1 | 1 |
| 20300002 | 0 | 2 | 0 |

In passive mode TIGERs called all nine boundaries once and credited all four goals to the correct team. The last seed tested a full goalless minute. Across the three matches, the observed calls arrived 0.36–0.44 simulation seconds after the archived crossing tick. This includes camera sampling, tracking and detector delay. The observer timestamp identifies the latest world frame at callback time. [Nine comparisons and all other detector reports](passive.json)

The teams played the same 4,500 control frames as the original matches. Ball and robot trajectories, roles and events matched exactly. The local referee immediately resets the field on a crossing. To give the external detector a view beyond that instant, each of the nine plays also ran independently from its seed to the exit. Webots then advanced the ball for one second with robot commands held at zero. The camera stream inserts those measured frames before the reset. The verifier checks that the first frame of each continuation agrees with the independent live rerun. A second UDP listener received all 4,725 detection frames from this experiment, each with a ball and ten identified robots. [Frame counts](udp-frames.json) · [Original match traces](../five-roles/matches)

The active film uses continuous native physics and the Game Controller's live broadcast. Webots emits ideal overhead detections in SSL-Vision format. The TIGERs vision filter tracks the ball and robots. TIGERs uses its own goal detector and its active game-event connection. The controller's automatic goal approval is enabled. A simulated table operator starts play and invokes the controller's Continue actions for the restart. The controller selects the awarded team and Webots places both teams in kickoff formation. The table script has no score-edit operation. [Table actions](../../scripts/autoref/table.mjs)

This fixture evaluates goals and boundary calls. The passive observer also reported five robot collisions and seven defense-area offenses. Those reports are retained in the evidence. The active fixture limits stoppages to goal and boundary events. The follow-up placement experiment trains the awarded side to collect and place the ball. Full foul handling and camera noise remain open. [Placement results](../../policies/placement_v1/README.md)

## Run the experiment

Use the modern stack's Python environment, Webots and a current JDK. Download and unpack the published AutoRef distribution and the official Game Controller for your platform. The active fixture also uses Node with built-in WebSocket support. [TIGERs 1.7.1](https://github.com/TIGERs-Mannheim/AutoReferee/releases/tag/1.7.1) · [Game Controller v3.23.1](https://github.com/RoboCup-SSL/ssl-game-controller/releases/tag/v3.23.1)

Run these commands from the modern stack directory. Set `TIGERS_DIST` to the unpacked directory containing `lib` and `config`. Put the Game Controller on your PATH as `ssl-game-controller` or pass its executable with `--game-controller`.

```sh
mkdir -p runs/autoref-classes
javac -cp "$TIGERS_DIST/lib/*" -d runs/autoref-classes scripts/autoref/EventObserver.java
python scripts/autoref/run_fixture.py passive runs/autoref-passive \
  --distribution "$TIGERS_DIST" --classes runs/autoref-classes
python scripts/autoref/run_fixture.py active runs/autoref-active \
  --distribution "$TIGERS_DIST" --classes runs/autoref-classes
python scripts/autoref/film.py runs/autoref-active runs/autoref-film
```

The passive fixture runs nine native continuations and three complete matches, then writes `passive.json`. The active fixture writes the original movie, pose trace, observer log, controller protocol and `active.json`. Use new output directories for each run. The observer logs the release's event protobufs and the corresponding tracker timestamps. Movie captioning uses FFmpeg and Pillow.

To send field geometry and detections to other SSL-Vision clients, pass `--ssl-vision 224.5.23.2:10006` to `scripts/run_five_roles.py`. The qualification uses a loopback UDP destination and the official framed AutoRef CI transport to advance the external detector with Webots. For a running CI instance, pass `--autoref-ci 127.0.0.1:10013`. Its filtered output can be sent to a tracker consumer using `--tracker-output HOST:PORT`. [League simulator interface](https://github.com/RoboCup-SSL/ssl-game-controller/blob/v3.23.1/doc/AutoRefCi.md)
