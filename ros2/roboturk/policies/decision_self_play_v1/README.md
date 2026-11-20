# Learned tactical decisions and self-play v1 — in development

A centralized neural manager chooses pass/receive, dribble or shoot, the acting
robot, receiving and dribble points, and goal-mouth aim. Frozen trained motion
skills execute its decisions; the old scripted target planners and handoff are
absent from deployed inference. The manager and both opponents will be trained
through alternating native self-play. Teacher logic is restricted to initialization
and labeled baselines. No new reliability result or demo is claimed yet.

[Protocol, fresh seeds and gates](../../docs/LEARNED_DECISIONS.md).
Local contract/regression checks:142passed,oneROS-onlyskip; Ruff clean. Native
teacher baseline completed57/60clean:28goals,29defensive stops,one possession loss,
one field exit and one collision. The first56/60panel is retained; a premature
acquisition measurement was corrected before training.300native demonstration
episodes and the first150mixedDAgger episodes completed. Existing mobile-self-play policies are the motion
skill/opponent starting point, not evidence for learned tactical decisions.

First fully neural manager development FAILED:52/60clean,4goals,48defensive stops,
six invalid-kick labels and two field exits. Every invalid-kick label followed an
already applied valid shot (empty retrigger while ball in flight); original records
remain. Goal-mouth target averaging and reset-state target errors are under review.
Teacher collection300games:172goals,109stops,19failures. The manager fit used71,913
native observations/100epochs; DAgger completed; results below. No self-play or qualification
claim yet. [Initial manager](artifacts/teacher/manager.zip),
[collection](artifacts/teacher-collection.json), [pure-neural outcomes](artifacts/teacher-development.json).

V1 DAgger improved to57/60clean:16goals,41stops,one empty-shot-retrigger label and
two real keeper field exits. All goals were close starts. The revisedv2contract
learns a discrete goalcorner and stores executed-action memory, while retaining
continuous receiving/dribble targets. Reset/switch-balanced supervised preparation
now gives27goals,30stops,three failures/60:midfield3goals,advanced7,close17. Two keeper
field exits and one timeout remain; no reliability claim.150fresh v2DAgger cases
completed before alternating PPO. [V2 preparation](artifacts/prepared-v2/training.json)
and [native outcomes](artifacts/prepared-v2-development.json). These changes include
a measurement correction for harmless empty post-shot retriggers; do not attribute
all count changes to learning. All earlier strict-monitor records remain.


V2 DAgger:32goals,21stops,seven failures/60 (two keeper field exits, two ball-outs,
three timeouts). Midfield5goals/15clean, advanced9goals/18clean, close18goals/20clean.
This candidate is not qualified. Receiving targets drifted or switched during failed
passes. V3 adds a learned target update/retain gate, with no scripted handoff; explicit
migration/refitting and fresh DAgger are running. No PPO or reserved result yet.
[V2 failed development](artifacts/dagger-v2-development.json).

V3 completed: prepared29goals/56clean; DAgger26goals/56clean, with one collision, one ball-out and two keeper field exits in60development games. Goals by family3/7/16; eight stable receptions. This remains unqualified. Alternating native PPO now starts from the final v3 DAgger model; transfer/finishing actors stay frozen.
