# Choosing a pass recipient in 3v3

Six robots play one-minute Webots matches on a 9 × 6 m field. When the carrier
has a passing opportunity, a shared linear scoring function evaluates both
teammates. Each candidate has six features measured relative to the carrier,
ball, goal and three opponents. These describe passing lane clearance, marking
distance, forward progress, range, turn angle and receiver orientation. Robot
IDs identify recipients and do not enter the score. Choosing deterministically
at deployment uses the same features as sampling a receiver during training.

Each seed starts near uniform and plays 20 native matches. The learning side
alternates colors and faces a sampled historical receiver policy. Both teams
sample receiver choices. The score-function policy gradient uses +1 for an actual
intended reception and −1 for an interception, timeout, restart or end of match
after a kicked pass. Only the learning side updates its six parameters after each
match. Its checkpoint joins the opponent pool every four matches.

The pass motor comes from the previous ten-skill release. The 3v3 experiment
trains recipient selection. Carrying, shooting, support, defensive coverage,
receiver motion and geometric velocity limits use explicit controllers. The
carrier can keep the ball when both lanes are crowded. The game uses a 12-second
possession clock and randomized restarts after expiration, a goal or a ball-out.
Each restart permutes the robot IDs and places a defender near one candidate
lane. All six robots move throughout each match.

An intended reception requires a real kick, 0.55 m of ball travel and at least
0.20 seconds of exclusive mouth possession by the selected robot within four
seconds. An opponent or the other teammate taking the pass ends the attempt.
The audit checks the outcome against 10 ms native position samples and checks
the selected teammate against the policy's recorded candidate scores.
[Protocol and test cases](protocol.json).

The main qualification is 32 held-out one-minute matches. A separate eight-match
open-play challenge extends the clock to 90 seconds and removes shot-clock
restarts. The two consecutive uncut one-minute demo matches use the same open-play
rules. Goal and ball-out restarts remain active in both formats.


To reproduce on the pinned Webots build, run the launcher from the modern project
root with the isolated environment active. It writes a fresh directory for each
experiment. Training requires the prior frozen pass motor. Qualification and the
demo also load the two released recipient selectors and check the frozen files.

```sh
python scripts/run_three.py --webots /path/to/webots --mode train --seed 901 --output runs/new-league-901
python scripts/run_three.py --webots /path/to/webots --mode train --seed 902 --output runs/new-league-902
python scripts/run_three.py --webots /path/to/webots --mode qualify --output runs/replay-3v3
python scripts/run_three.py --webots /path/to/webots --mode open --output runs/replay-3v3-open
python scripts/run_three.py --webots /path/to/webots --mode demo --output runs/replay-3v3-demo
```
