# Converting the opening recovery in 3v3

We examined all 40 opening contests from the second recovery team's held-out Webots test before training the next policy. The ball and all six robot poses were recorded at 100 Hz. Turnover time begins when one of the contestants has controlled the ball alone for 0.20 seconds. A pass in flight counts as a live attack until an opposing mouth touches the ball or the referee stops play.

![Paths of the ball from recovery to the end of an unsuccessful attack](../../media/recovery-transition/previous-losses.png)

| Opening outcome | Games | Median time after recovery |
| --- | ---: | ---: |
| Scored from the first recovery | 17 | |
| Opponent touched the ball before the first kick | 8 | 0.07 s |
| Opponent touched the ball after a pass | 4 | 0.65 s |
| Opponent touched the ball after a shot | 2 | 4.72 s |
| Ball went out after a shot | 5 | 5.74 s |
| Neither contestant established a verified recovery | 4 | |

Six of the eight pre-kick turnovers happened within 0.08 seconds. The four pass interceptions ended near midfield, with median attacking-frame x of 0.09 m. The native trajectories show shots going out both near the goal and over the touchline. [Every opening, outcome and location](predecessor-losses.json) · [Original traces](../recovery_attack_v2/qualification/trained)

An additional check covered every spontaneous recovery in the previous team's twelve one-minute games. Of eleven recoveries, four led to goals and four were lost to an opponent before a kick. One ended with an opponent touch after a pass, one after a shot, and one reached the time limit before a kick. [Ordinary-play loss records](ordinary-losses.json)

## Self-play

During the transition from joint to sole possession, the controller tracks ball velocity and centers the moving ball in the robot's front mouth. A carrier can rotate to aim while translating. The learned team policy chooses a pass, dribble or shot using the available goal opening, pressure, alignment, distance to goal and the lead-pass route. Teammates clear the shooting lane or move to the intended receiving point. Both colors learn from actual Webots games, with normal opponent commands, kick speeds and contact physics.

The policy search tries several combinations of ball tracking, dribble speed, escape velocity and decision weights on the same layouts. Each candidate earns a point only for a physically verified goal from the opening contest. Ties favor successful passes and then controlled carries. Every game in the 40-start qualification gets an independent Webots process, with the same field, contest distribution and scoring audit as the previous team. [Registered protocol](protocol.json)

In the first league, each of six policies played 48 training starts. Their opening-goal counts were 37, 30, 37, 32, 37 and 32. The three policies tied at 37 scored 18, 17 and 23 dribble-assisted goals respectively. The top two advanced alongside four variations that give more weight to an available pass. [First league returns](initial-returns.json) · [Training records](initial-training.json.gz)
