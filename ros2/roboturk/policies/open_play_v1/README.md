# Open-play recovery goals in 3v3

The new team scored **3.44 recovery goals per 90-second match** in self-play. The previous 31/40 opening team scored **0.56**. Both played the same 16 ordinary starts in independent native Webots processes. [All 16 pairs](comparison.json)

| Full-match measurement | New team | Previous team |
| --- | ---: | ---: |
| Open-play recoveries | 176 | 54 |
| Verified recovery goals | 55 | 9 |
| Recovery conversion | 31.25% | 16.7% |
| Recovery goals with a completed pass | 17 | 3 |
| Recovery goals in the final 45 seconds | 17 | 7 |
| All goals, including possessions that began at restarts | 58 | 13 |

The difference was 2.88 recovery goals per match. The new team produced more recovery goals on 15 starts and the teams tied on the remaining start. A paired bootstrap over the 16 starts gave a 95% interval of 2.00 to 3.81 extra goals per match. Each start gets equal weight in that comparison. [Per-match results, recovery types and failure counts](comparison.json)

In 16 head-to-head matches, the teams played both color assignments on each of eight starts. The new team won 6, drew 6 and lost 4. It scored 14 goals from 37 recoveries. The previous team scored 9 from 37. The overall score across those matches was 16–10. All 48 evaluation matches ran for 90 seconds. The 100 Hz monitor checks a robot-center envelope of ±4.41 m by ±2.91 m, accounting for the robot’s footprint. It flagged a boundary excursion in one trained mirror match (499 samples), one previous mirror match (1,981 samples) and one mixed match (1,981 samples). It also recorded 285 samples with robot-center separation below 0.18 m across the previous-team and mixed matches. In the filmed match, Blue 1 crossed the envelope from 74.00 to 78.98 s and reached |x| = 4.468 m. None of the 87 verified scoring sequences overlapped a boundary or separation flag. [Traces and event logs for all 48 matches](evaluation/)

All 55 trained mirror goals followed contested-ball wins. Its three clean takeaways produced no goals on this panel. The most common failed outcome was another opponent touch, ending 86 of the 176 recoveries.

## What the team learned

Both sides kept the previous passing, shielding, finishing and contested-ball motor skills. The new policy also activates the counterattack after a clean turnover. The nearest available defender pursues the rolling ball at an extrapolated position. Full-match self-play selected the pursuit speed (0.88 m/s), extrapolation time (0.16 s), and attacking parameters governing pass preference, advance speed and shooting clearance. The goal was to win the ball in live play and finish before another opponent touch.

Parameter search compared the incumbent and three candidate configurations in 64 complete 75-second games on eight training starts. The league included self-play mirrors and rotating opponents. It selected candidate 1 by recovery goals per team appearance plus its fraction of successful recoveries. The previous team was the reference. The selected weights and source hashes were frozen before the reserved evaluation. The training files include the original events and outcomes from all 64 games. Their first trajectory sample already contains 10 ms of movement. For the held-out comparison we recorded the complete Webots state before the first command and required it to match exactly for each seed. [Training schedules, parameters and all game logs](training.json.gz) · [Frozen policy](policy.json) · [Code and weights checksums](freeze.json)

## What counts as an open-play recovery

A player must win 0.20 seconds of sole roller control after an opponent controlled the ball or after a sustained two-team contest. Start and restart possessions give no credit. A scoring chain ends at the next opponent touch or stoppage. To count, the final carrier must shoot the ball fully across the opponent’s goal line inside the posts. Every pass in the chain must travel at least 0.55 m and reach its intended teammate for 0.20 seconds of sole possession. This also credits a winner who carries and shoots directly. The simulation uses ground-truth state, ideal body-velocity commands and spring dribblers. The earlier 31/40 result was measured on a different, 18-second opening task. [Registered protocol](protocol.json)

## Watch and reproduce the match

Yellow 2 wins the ball at 55.20 s and plays it to Yellow 0. Yellow 0 finds Yellow 1, who scores at 62.52 s. The clip comes from evaluated seed 16300000. The replay selection favors a completed pass, then a recovery after 45 seconds, then the earliest seed and recovery. In the paired clip, the two matches share the same kickoff and run independently thereafter. Both recorded matches reproduced all 9,000 physical samples and event logs from their respective evaluations. [Goal GIF](../../media/open-play/open-play-recovery.gif) · [Matched-start GIF](../../media/open-play/same-start.gif) · [Full 90-second video](../../media/open-play/open-play-match.mp4) · [Recording checksums](../../media/open-play/recording.json)

Run from the modern project in its Python environment with the native Webots runtime.

```sh
python scripts/train_open_play.py train runs/new-open-training
python scripts/train_open_play.py evaluate runs/new-open-evaluation
python scripts/summarize_open_play.py runs/new-open-evaluation runs/new-open-comparison.json
python scripts/train_open_play.py record runs/new-open-demo --from-evaluation runs/new-open-evaluation
python scripts/train_open_play.py record runs/new-open-prior --previous --from-evaluation runs/new-open-evaluation
python scripts/record_open_play.py runs/new-open-demo media/new-open-demo --previous runs/new-open-prior
```
