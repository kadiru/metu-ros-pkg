# Continuous 2v2 — training in progress

**This unfiltered experiment is not qualified. There is no new reliable continuous-match demo yet.**
The separate [v2 experiment](../continuous_play_v2/README.md) adds explicit translation
safety and retains an independent protocol; its results cannot establish entirely
neural collision avoidance.
The previous qualified [learned-decision release](../decision_self_play_v2/README.md)
and all 89 files in its frozen runtime remain unchanged.

The new policy controls both teams through 45-second matches. Interceptions, loose
balls and missed shots continue play. Goals and ball-outs cause explicit, logged
referee restarts. Seven learned behavior scores select distinct neural motion
heads: recover, dribble, pass, shoot, press, cover and support. The current v4
contract has 122 observations and 35 outputs, including measured local geometry,
ball-distance comparisons, score and clock. Deployment has no teacher, tactical
planner or collision-avoidance override.

This uses native Webots with ground-truth state and approximate dynamics. It does
not establish hardware performance, official SSL match compliance or a complete
live ROS graph.

## Independent training and retained failures

Seeds **611 and 622** have separate initializations and corrective datasets. Seed
611 was selected as the primary release before evaluation. Shared initial teacher
collection: 30 native matches, 135,224 robot observations, 29 goals, 80 passes, no
contacts or field exits. These are analytic demonstrations, not neural results.

| Neural preparation | Seed 611: completion / goals / contact time / field-exit matches | Seed 622 |
| --- | --- | --- |
| Initial 80 epochs | 0/6 · 0 · 52.75% · 6 | 0/6 · 0 · 30.24% · 6 |
| Additional 320 epochs | 6/6 · 1 · 17.96% · 5 | 5/6 · 0 · 18.77% · 6 |
| First mixed DAgger, 320 epochs | 6/6 · 2 · 30.47% · 4 | 6/6 · 2 · 20.20% · 4 |
| V3 local geometry, 320 epochs | 4/6 · 0 · 18.82% · 4 | 6/6 · 3 · 10.82% · 6 |
| Corrected roles and restored spacing, 320 epochs | 5/6 · 2 · 5.81% · 5 | 5/6 · 1 · 7.05% · 5 |
| Full neural DAgger, 3-game diagnostics | 3/3 · 2 · 6.41% · 0 | 3/3 · 1 · 3.76% · 0 |
| Canonical labels and precision fit | 6/6 · 7 · 2.79% · 0 | 6/6 · 7 · 2.24% · 1 |

All checkpoints, metadata and raw development outcomes are retained under
`preparation/` and `development/`. The intermediate 0.30 m spacing experiment also
remains: it stalled capture, so its clean contact count is not a success claim.

The training teacher's original local hysteresis could assign both teammates to
cover. Native traces exposed their convergence on the same point. The teacher
now assigns one nearest chaser; the neural observation includes ball-distance
comparisons. In seed 611's sampled trace, cover/cover contact frames fell from 288
to zero. Other contacts and field-edge failures remain.

Recorded native training observations were reconstructed and relabeled with the
corrected teacher. This produces new training labels, not new physics transitions.
Original datasets and labels remain intact, with source snapshots and hash receipts.
A wider 0.30 m separation target prevented opposing chasers from reaching a loose
ball. Restoring 0.23 m while retaining the role correction produced a paired teacher
pilot of 3/3 complete matches, four goals, eight passes and six receptions, without
contacts or field exits.

The second DAgger stage used **100% neural actions**, with the teacher supplying labels
only, on 30 fresh training matches per seed. Refitting balanced field margins, nearby
robots and close ball-distance comparisons. Both initial full-motion PPO leagues
completed two generations: 131,072 native steps and 128 accepted updates in total.
Their final development contact rates exceeded the 2% gate; both attempts are retained.
Fresh isolated leagues now train only behavior-score readouts while freezing all
neural motion functions. These use the symmetric kick-arbitration fix described
below. Reserved qualification has not started.

## Evaluation contract

[The predeclared protocol](protocol.json) fixes six 20-match reserved panels: self,
historical and independent-seed opponents. Each training seed must pass all gates
across 60 games. The fresh demo retains three consecutive 45-second matches with
611 against 622. Reserved and demo seeds remain unused.

Metric version 2 requires exclusive stable acquisition. Receptions must follow an
actual teammate pass within five seconds and at least 0.5 m displacement, with no
intervening opponent control. Loose-ball recovery, support and cover movement are
reported separately. Earlier raw results retain their original metrics. Unfinished
matches receive no win or draw credit in current reports.

The independent auditor checks whole-ball goals, reception provenance, restart
records, continuous commands and controlled counterattacks. Runtime/model hashes
must be published and verified before reserved evaluation. Native camera preflight
now shows the entire field and all four robots; its videos are development checks,
not a reliable-policy demonstration.

[Runtime, training protocol and history](../../docs/CONTINUOUS_PLAY.md).

The avoidance teacher also used color-dependent iteration order in crowded states.
It now processes teammate, opponent 0 and opponent 1 consistently with the actor's
canonical observations. Relabeled native DAgger states and a lower-rate precision
fit retain their exact source and data hashes. No new synthetic transitions are
claimed. Self-play penalizes both new contact events and sustained contact time;
scoring receives more reward than simply holding possession. Learner roles rotate
through shuffled cycles of all four robot/color assignments.

## Continuous-only kicker correction

A read-only audit of 142 development matches found five pairs of kicks applied at
the same physics instant. The legacy backend allowed the last robot in iteration
order to overwrite the ball velocity. A separate continuous backend now applies
at most one kick per 5 ms sample: the eligible dribbler mouth closest to the ball
wins; exact distance ties defer the kick. Native mirrored/color-swapped contact
preflights passed all eight cases. Subsequent ODE contacts can still break perfect
ties numerically. Kick times and arbitration decisions are logged and independently
audited. Previous frozen backends and results remain unchanged. This is still an
approximate velocity-kick model.

Both Gaussian decision-only leagues also completed 131,072 native steps and 128
accepted updates in total. Final development: 611 completed6/6 with4goals,
6receptions,4.25%contact time and1field-exit match;622 completed6/6 with5goals,
6receptions,2.90%contact time and1exit. Motor functions stayed byte-identical.
Only one deterministic mode choice changed across the two611round probe sets;
none changed for622. These are retained failures, not an RL improvement claim.
Together with the earlier full-motion attempt, this is262,144 native PPO steps.
