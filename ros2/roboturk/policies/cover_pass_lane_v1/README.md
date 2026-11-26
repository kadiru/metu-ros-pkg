# Learned passing-lane coverage v1

**200/200 reserved native Webots cases passed**, with zero robot collisions. The
learned defender stayed within **0.08 m of the assigned moving lane midpoint for
3 continuous seconds** in every case. Mean completion: **3.4481 s**;
Wilson95% interval98.12–100% for this distribution. 137,924 checks
at5ms physics substeps. [Raw outcomes](qualification.json) · [Summary](qualification-summary.json).
The [native demo](../../media/cover-pass-lane-demo.mp4) contains all **3/3** fresh cases.

This evaluates geometric lane coverage. It does not demonstrate a physical save,
an intercepted pass or match-level defensive strength. The separately qualified
[block/recover policy](../block_shot_v1/README.md) measures physical retention.
Neither learned defender replaced the fixed opponents in attacking qualification.

## Contract and task

`roboturk.cover.v1`:16 observations, two learned attack-frame translation outputs,
scaled to1m/s per axis, fixed heading. Shared `defense_observation`, `defense_command`
and `DefenseCheckpoint.command` are used for training and inference. No teacher
fallback. The tactical layer supplies target position, velocity and observed dwell.

The ball is stationary1.8m behind midfield, with randomized lateral position. An
opponent1.8m ahead moves laterally with sinusoidal amplitude0.35–0.65m, angular
frequency0.3–0.7rad/s and randomized phase. The target is the ball-to-opponent
midpoint; choosing that threat and midpoint is scripted. The actor learns movement
and tracking. Initial defender offsets are up to0.4m longitudinal and0.5m lateral.
Attack direction is randomized. Every5ms sample must satisfy the0.08m error bound
through the final3s; endpoint-only contact with the target is insufficient.

## Training and freeze

CPU seed117, one Torch thread, MLP128×128. Teacher200/200 and mixed DAgger100/100.
BC, refit and32,768-step anchoredPPO each scored60/60 development cases. The faster
BC actor was selected;32/32 actor updates accepted. No PPO success-rate improvement
is established. This is one training seed, not a replication study.

Frozen at`2026-11-19T04:46:12.540691-08:00` before reserved evaluation. Teacher1100000–1100199;
DAgger1120000–1120099; development1110000–1110059; qualification1150000–1150199;
demo1160000–1160002. All stage checkpoints/results remain here; larger datasets/logs
remain in ignored`runs/cover-v1`.

Checkpoint SHA256:`afc9b76860cdd5b561ff370de596edf4d62cd2f403594efdfbd5cae6670d1b1e`.
CMDragons' threat-lane coverage discussion inspires the behavior; see
[paper provenance](../../docs/BEHAVIOR_SOURCES.md). Ground-truth, approximate native
Webots body-velocity and contact physics; no noisy vision, calibrated hardware,
free-ranging opponents or official-match qualification.
