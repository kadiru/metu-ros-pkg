# Protecting the ball winner’s first touch

On 40 fresh, independently launched native Webots openings, the frozen 3v3 team completed **31 recovery, lead-pass, shield and goal sequences**. The original winner lost possession before passing in just **one opening**. On the same starts, the previous team completed 24 sequences and lost possession before passing three times. Blue scored 16 of the new goals and Yellow scored 15. [Matched outcomes](comparison-9.json)

| Opening outcome | Trained team | Previous team |
| --- | ---: | ---: |
| Confirmed recovery | 37 | 36 |
| Pressured receive, shield and goal with continuous carrier possession | 31 | 24 |
| Lost sole control before the first pass | 1 | 3 |
| Opening pass intercepted | 0 | 1 |

Every opening counts. The scorer must receive a lead pass around a blocked lane, protect the ball from a closing defender for at least 0.20 seconds, and finish on the same possession. The original winner must keep sole control through release. The evaluator checks possession every 10 ms. The registered threshold was 30 goals and at most two pre-pass losses. Three openings ended without a confirmed recovery. In three others a teammate received the pass but did not finish on that possession. Two receivers scored with insufficient closing pressure to qualify. In the last case the opponent touched the ball before the winner could pass. The contact monitor counted 0 robot-center separations below 0.18 m and 0 field exits. [Protocol](protocol.json) · [New team, events and 100 Hz traces](qualification-9/) · [Matched previous team](control-9/)

## The five historical losses

The original ball slipped sideways in four cases and out the front in one. The original 18-second state and event records replayed exactly. Each rematch changed only the historical winner’s team policy and retained the previous opponent. The comparison clip shows the fastest of the five original losses. Blue 2 lost the ball 0.01 seconds after securing it. In the rematch, Blue 2 carried it 0.44 m and led Blue 0 by 0.40 m. Blue 0 kept its body between the ball and the closing marker, then scored at 5.36 seconds. Each side reproduced all 1,800 physical samples of its evaluated game. [Original measurements](original-losses.json) · [Rematch outcomes](historical-9/) · [Side-by-side GIF](../../media/recovery-touch/first-touch-comparison.gif)

| Historical seed | Original winner | Rematch winner | Rematch outcome |
| ---: | --- | --- | --- |
| 14301004 | yellow 2 | yellow 2 | no opening pass |
| 14301018 | blue 2 | blue 2 | pressured shield goal |
| 14301026 | yellow 1 | yellow 1 | no opening pass |
| 14301031 | blue 2 | blue 2 | pressured shield goal |
| 14301033 | yellow 2 | yellow 2 | no uninterrupted finish |

## Training and replay

The skill is a parameterized feedback policy. Native self-play selected its coefficients by rewarding qualified finishes and subtracting two points per lost first touch. The search tested when to correct a slipping ball, how to carry it clear of the opposing roller, and which maneuver to use while both dribblers touched the ball. One experimental team learned which maneuver to use when two rollers had the ball, using the eventual recovery and scoring outcome as feedback. The carrier follows its usual pass or dribble until the observed ball velocity and the requested roller velocity predict a slip. It cushions the ball or moves sideways if the opposing roller also approaches. After the receiver shields, the policy also predicts whether the ball will slip while the robot pivots to shoot. When it predicts a slip, it adjusts the roller motion during the turn. The receiver also matches mouth velocity to the rolling ball while pivoting into the shield. The team policy selects an outlet and leads the moving receiver. Both sides use the same policy in the fresh benchmark. The five historical starts are diagnostic and excluded from qualification. [5526 self-play training games and candidate schedules](training/) · [Final weights](planned-policy-9.json) · [Code and weights checksums](planned-freeze-9.json)

Each frozen team had a different 40-opening panel. Its previous-team control used exactly the same initial states. Teams 8 and 9 were frozen together before either was evaluated. Both met the threshold. The registered selection rule chose team 9 by its higher qualified-goal count. [Paired test registration](paired-challenge.json)

| Frozen team | Qualified goals / 40 | Pre-pass losses | Previous goals / 40 | Previous pre-pass losses |
| --- | ---: | ---: | ---: | ---: |
| [Team 1](attempt-1.md) | 26 | 9 | 25 | 5 |
| [Team 2](attempt-2.md) | 28 | 3 | 22 | 4 |
| [Team 3](attempt-3.md) | 27 | 5 | 28 | 5 |
| [Team 4](attempt-4.md) | 29 | 4 | 30 | 7 |
| [Team 5](attempt-5.md) | 30 | 3 | 27 | 5 |
| [Team 6](attempt-6.md) | 29 | 1 | 29 | 3 |
| [Team 7](attempt-7.md) | 25 | 6 | 20 | 8 |
| [Team 8](attempt-8.md) | 30 | 2 | 26 | 3 |
| [Team 9](attempt-9.md) | 31 | 1 | 24 | 3 |

To study the transition from shielding to shooting, we replayed four openings where the pressured receiver took the pass and failed to finish. We tried five receiver motor settings on each start. All 20 trajectories were identical to their originals through release. Two motor choices converted the same previous miss, seed 15305034, into a verified goal. Across all 40 of those previous starts the second alternative scored 25, down from 29, with the same one pre-pass loss. All 39 openings with an audited kick matched through that kick. [Four missed finishes](training/matched-finish-misses.json) · [Full receiver regression](training/receiver-regression.json)

Fresh qualification used seeds 15308000–15308039. Run the following from the modern project in its Python environment with the pinned native Webots runtime. Each destination must be new. The three-opening demo scored 1/3 and a separate launch reproduced all 5,400 physical samples and all game records. [Native demo MP4](../../media/recovery-touch/first-touch.mp4) · [Uncut GIF](../../media/recovery-touch/all-openings.gif) · [Demo traces](demo-9/)

```sh
python scripts/train_clean_recovery.py qualify runs/verify-touch --version 9
python scripts/train_clean_recovery.py control runs/control-touch --version 9
python scripts/compare_recovery_touch.py runs/verify-touch runs/control-touch runs/touch-comparison.json
python scripts/train_clean_recovery.py demo runs/watch-touch --version 9
python scripts/record_blocked_outlet.py runs/watch-touch media/local-touch
```

These measurements cover the opening possession of an 18-second 3v3 simulation. Webots supplies ground-truth state and idealized drive, kick and roller actuation. The marker actively pursues the receiver under the same rule in both panels.
