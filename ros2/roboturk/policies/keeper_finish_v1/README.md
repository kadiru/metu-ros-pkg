# Keeper-aware finishing v1 — qualification pending

This new32-input/four-output128×128 MLP controls body velocity, rotation and shot
release. It composes with the frozen defended-transfer v3 actor. The passer begins
holding the ball; both opponents remain active throughout the complete play.

The play selects a flank receiving point and hands off after measured stable
reception. A separate scripted finishing tactic selects a parking point1.0m from
the goal line and0.18m to the ball's initial side, then aims toward that side's
0.43m goal corner. Once settled, it latches the opposite corner when the keeper
has moved toward the lure, or after1.2s settled dwell for a ball-line keeper.
The neural policy produces every attacker motion and release command during
finishing; no teacher fallback or action shield. Kick speed3.5m/s and dribbler
mask are fixed interface rules. Progress is measured to the goal center, so an
updated aim point cannot manufacture carried distance.

The additional opponent is a delayed, acceleration-limited analytic goalkeeper:
speed0.65–0.95m/s, acceleration2–3m/s², reaction0.08–0.14s, randomized orientation-
tracking or ball-line style, incoming-shot prediction in both. The existing
midfield defender is unchanged. These are bounded opponents, not adversarial
self-play or unrestricted competitive defenders.

Training:300 native Webots demonstrations and150 mixed DAgger episodes all scored.
Both cloned and refitted actors scored60/60 development episodes; the faster clone
was selected for anchored PPO. All65536 PPO steps completed, with64 accepted actor updates; final standalone development60/60, zero losses, mean8.823s. No success-rate improvement over cloning was observed. A None-seed reset bug interrupted PPO before its
first update; refinement resumed from retained data/checkpoint after a replay-tested
fix. Complete-play development scored120/120 with zero losses/collisions, mean14.9725s. Checkpoints and runtime hashes frozen04:19:46 Pacific before reserved evaluation. Qualification and demo are pending; development is not a reliability claim.
No incremental PPO benefit is established merely by running PPO.

See [protocol and reproduction](../../docs/KEEPER_TRAINING.md) and
[team-paper provenance](../../docs/BEHAVIOR_SOURCES.md). ER-Force's keeper/aim
analysis inspires the tactic; our learning/control implementation is separate.
Initial failed baselines remain here, with the initial keeper coordinate bug
explicitly documented in the protocol. No tuning will use this version's reserved
qualification seeds950000–950199 or demo seeds960000–960005.

Ground-truth observations and approximate Webots ideal body-velocity/ODE-contact,
spring dribbler and edge-triggered velocity-kick dynamics. No calibrated wheels,
rollers, kicker, vision/radio latency, hardware or official-match qualification.
The native Supervisor invokes the core play; a complete live ROS/Webots graph
with applied-kick feedback still needs separate validation.
