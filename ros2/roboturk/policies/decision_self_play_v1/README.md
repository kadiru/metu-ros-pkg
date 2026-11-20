# Learned tactical decisions and self-play v1 — in development

A centralized neural manager chooses pass/receive, dribble or shoot, the acting
robot, receiving and dribble points, and goal-mouth aim. Frozen trained motion
skills execute its decisions; the old scripted target planners and handoff are
absent from deployed inference. The manager and both opponents will be trained
through alternating native self-play. Teacher logic is restricted to initialization
and labeled baselines. No new reliability result or demo is claimed yet.

[Protocol, fresh seeds and gates](../../docs/LEARNED_DECISIONS.md).
Local contract/regression checks:124passed,oneROS-onlyskip; Ruff clean. Native
teacher baseline completed57/60clean:28goals,29defensive stops,one possession loss,
one field exit and one collision. The first56/60panel is retained; a premature
acquisition measurement was corrected before training.300native demonstration
episodes and150mixedDAgger episodes are now being collected/trained. Existing mobile-self-play policies are the motion
skill/opponent starting point, not evidence for learned tactical decisions.

First fully neural manager development FAILED:52/60clean,4goals,48defensive stops,
six invalid-kick labels and two field exits. Every invalid-kick label followed an
already applied valid shot (empty retrigger while ball in flight); original records
remain. Goal-mouth target averaging and reset-state target errors are under review.
Teacher collection300games:172goals,109stops,19failures. The manager fit used71,913
native observations/100epochs; DAgger is still running. No self-play or qualification
claim yet. [Initial manager](artifacts/teacher/manager.zip),
[collection](artifacts/teacher-collection.json), [pure-neural outcomes](artifacts/teacher-development.json).

V1 DAgger improved to57/60clean:16goals,41stops,one empty-shot-retrigger label and
two real keeper field exits. All goals were close starts. The revisedv2contract
learns a discrete goalcorner and stores executed-action memory, while retaining
continuous receiving/dribble targets. Reset/switch-balanced supervised preparation
now gives27goals,30stops,three failures/60:midfield3goals,advanced7,close17. Two keeper
field exits and one timeout remain; no reliability claim.150fresh v2DAgger cases
are running before alternating PPO. [V2 preparation](artifacts/prepared-v2/training.json)
and [native outcomes](artifacts/prepared-v2-development.json). These changes include
a measurement correction for harmless empty post-shot retriggers; do not attribute
all count changes to learning. All earlier strict-monitor records remain.
