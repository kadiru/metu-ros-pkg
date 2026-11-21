# Winning the ball under pressure in 3v3

RoboTurk learns how to keep the ball in its own dribbler while pulling or rolling it beyond the opponent's dribbler. A second policy moves a teammate into a receiving position. The frozen 3v3 recipient policy can choose either teammate for the pass.

## Results in 36 held-out native Webots matches

| Team configuration | Seconds jointly holding the ball | Recoveries / contests | Recovery-to-pass sequences | Games with a sequence | Games with a late reception | Intended receptions / attempts |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Released vs. released | 318.80 | 0 / 7 | 0 | 0 / 12 | 6 / 12 | 105 / 150 |
| Trained vs. trained | 16.58 | 16 / 16 | 10 | 7 / 12 | 12 / 12 | 136 / 200 |
| Trained vs. released | 41.19 | 9 / 11 | 6 | 5 / 12 | 11 / 12 | 131 / 190 |

The new policies cut time in double possession by **94.8%**. Five of the ten transfers in trained self-play reached the teammate designated as the outlet during the contest. In the other five, the recipient selector chose the other teammate after recovery. In mixed games the trained side completed all six sequences, including two to the designated outlet. A late reception means a completed pass in the final 30 seconds. Across 324,000 physics samples the audit observed zero robot field exits or inter-robot separations below 0.18 m. [Game-by-game results and checks](qualification/audit.json) · [Physical traces](qualification) · [Development outcomes](development.json)

## What changed on the field

The first version reduced time spent stuck but often freed the ball for the other team. The new motor policy observes where the ball sits inside **both** robot mouths. It uses those offsets, the robots' relative heading and the time spent contesting the ball to score twelve short movements. The available actions include pivoting about the dribbler and pulling the ball toward the nearer edge of the opposing mouth. Each robot samples its action from the trained scores using the match seed. Independent choices help two identical policies escape a repeated mirrored movement. The outlet policy scores seven receiving locations using field position, defender geometry and the teammate's travel distance. The timing trigger, bounded maneuvers and remaining 3v3 tactics are explicit controllers.

The division of labor comes from CMDragons' primary attacker and support attackers. In their 2015 offense, the primary attacker could protect the ball while a teammate searched its assigned zone for a receiving position. RoboTurk uses that coordination idea and trains its motions and outlet positions from its own play. [CMDragons 2016 Extended Team Description Paper, section 3.3](https://ssl.robocup.org/wp-content/uploads/2019/01/2016_ETDP_CMDragons.pdf)

## Training and controls

Two 96-episode native leagues trained the initial motor and outlet policies. The first candidate reduced joint possession but missed the reserved passing targets. Fitting returns separately against the released and historical opponents improved its choice of moves. Deterministic play could still repeat a mirrored move on both teams. Sampling broke that symmetry in development. Ordinary development games exposed a second problem when the ball sat off-center. [First candidate and failures](../contest_outlet_v1/README.md) · [Second league](league-two) · [Fitted return model](return-fit)

The final league used 21 physical contact poses from ordinary development games. It transformed the poses and varied the robot IDs, team assignments and field orientation. Over 256 eight-second episodes, the learning side faced the released system 128 times, the earlier sampled policy 64 times and historical checkpoints 64 times. It explored short maneuvers and fitted their returns to ball position and opponent geometry. Sole recovery earns one point, a subsequent verified transfer adds five, and losing the contest costs one. The outlet policy and pass motor come from the preceding leagues. Across these exploration games the learner recovered 98 times and completed 49 recovery-to-pass sequences. Replay recovered the final weights from the episode records. [Training poses](training-poses.json) · [All 256 episodes](jaw-training) · [Replay audit](training-audit.json) · [Selected policy](policy.json)

A counted sequence starts with at least 0.20 seconds of simultaneous opposing control. One of the original contestants must hold the ball alone for 0.20 seconds within three seconds of leaving the tie, then kick the ball within eight seconds. Its selected teammate must control it alone for 0.20 seconds after at least 0.55 m of travel, within four seconds of the kick. Opposing contact, an intervening kick or a referee restart invalidates the transfer. The audit reconstructs these events from the 100 Hz physical traces.

Each 90-second comparison runs in a fresh Webots process. The same 12 ordinary starts are used for released versus released, trained versus trained and trained versus released, alternating the trained team's color. All three configurations must match at the first physical frame to within 1e-7. Matches have no shot clock. [Qualification thresholds and seed ranges](protocol.json) · [Runtime and policy freeze](freeze.json)

## Replay

From the project directory, activate its Python environment and point the launcher at the Webots binary with `--webots /path/to/webots` if needed. The runs use Webots R2025a nightly dated November 17, 2026. The simulator uses ground-truth state, ideal body velocity, a spring dribbler and fixed kick power.

```sh
export PYTHONPATH="$PWD/src/roboturk_core"
python scripts/fit_contest_returns.py runs/refit policies/contest_outlet_v1/training policies/contest_outlet_v2/league-two
python scripts/run_outlet_league.py train_jaw runs/new-jaw-league --model runs/refit/policy.json
python scripts/run_outlet_league.py qualify runs/new-qualification
python scripts/run_outlet_league.py demo runs/new-demo
python scripts/build_contest_preview.py runs/new-demo --media runs/new-media
```
