# Learned block and recover v1

**200/200 reserved native Webots shots intercepted and retained**, with no goal
conceded or robot collision. Every success includes at least **0.5 s continuous
measured possession**. Mean completion2.2179s; Wilson95% interval98.12–100% for this
reachable-shot distribution. Qualification checks88,716 physics samples at5ms.
Both goal directions:99 and101 cases. [Raw outcomes](qualification.json) ·
[Summary](qualification-summary.json) · [Three fresh native demo cases](../../media/block-recover-demo.mp4), **3/3**.

This is a separate defensive micro task. It did not replace the analytic keeper
or defender in the attacking play's199/200 qualification.

## Contract and tactical context

`roboturk.block.v2`:16 observations, two learned attack-frame translation outputs,
scaled to1.2m/s per axis. Heading is fixed toward the field and dribbler duty is1.
The neural actor controls motion; no analytic motion fallback runs at inference.
A tactical target initially predicts where the rolling shot crosses the assigned
line. Measured entry into the dribbler capture region switches the target to ball
recovery, bounded to0.6m ahead of that line. This target selection is scripted.
`DefenseCheckpoint.command` shares the environment's observation/action code and
accepts an assigned target, target velocity and observed stable dwell.

Initial line is0.18m inside the goal line. Rolling shots begin1.8–2.8m away at
1.5–2.5m/s, with target y±0.32m and guard lateral offset up to0.25m. Randomized
attacking direction; approximate ideal body velocity, ODE contacts and spring
capture dynamics. This is a reachable-shot capture curriculum, not unrestricted
goalkeeping, calibrated hardware or full-match defensive qualification.

## Training, freeze and failures retained

CPU seed107;200/200 teacher and100/100 mixed DAgger episodes succeeded. BC, refit
and32,768-step anchored PPO each scored60/60 development cases. The faster DAgger
actor was selected;32/32 actor updates accepted. No PPO success-rate benefit is
established. MLP128×128; demonstration and regression data remain in ignored
`runs/block-v1`, with all stage checkpoints/results retained here.

The initial lateral-only teacher retained23/40 shots;17 rebounds timed out.
[All original outcomes](teacher-lateral-only.json) remain. Adding longitudinal
recovery changed the untrained v1 action contract to v2; shot distribution and
retention success criterion stayed fixed. Revised teacher40/40 before training.

A stale one-axis loader guard stopped the first qualification launch **before any
reset or reserved observation**. Its [traceback](qualification.error.txt) and
[original freeze](freeze-before-loader-fix.json) remain. After correcting the guard
and checking actual-checkpoint inference parity, the unchanged model was refrozen
at04:40:51 Pacific, November19,2026. [Current freeze](freeze.json).

Teacher1000000–1000199; DAgger1020000–1020099; development1010000–1010059;
qualification1050000–1050199; demo1060000–1060002. Reserved seeds were used once
with the frozen control implementation. One training seed, no replication claim.

Checkpoint SHA256:`1a9bcb7a8a591dbe32a81342c0bdc280bcebc7472c2d7301393feebae62aefb9`.
CMDragons' threat-based defense inspired this task decomposition; see
[paper provenance and behavior contracts](../../docs/BEHAVIOR_SOURCES.md).
