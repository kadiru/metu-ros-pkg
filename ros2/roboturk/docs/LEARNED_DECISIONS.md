# Learned tactical decisions and self-play v1

Started November19,2026 America/Los_Angeles. This is a new experiment; the previous
mobile-self-play release remains available at commit76679c00 and is not evidence
for learned target/option selection. Do not modify its policy artifacts.

## Contract and scope

A centralized neural manager chooses pass/receive, dribble or shoot, the acting
blue robot, a receiving point, a dribble setup point and goal-mouth aim. The same
manager runs throughout an episode. No scripted receiving-point planner, finish
phase threshold or reception-triggered handoff runs in deployed inference.
Physical possession/dwell, applied kicks and previous decisions are observations;
they do not select the next behavior. Discrete choices use actor-output argmax;
continuous targets decode within the field/goal geometry. Fixed learned transfer
and finishing networks supply motion and precise release timing. Referee/visibility
stops, actuator bounds, kick speeds and each skill's actuator semantics remain.
The manager is trained along with defender and goalkeeper against historical
snapshots. Low-level motion skills remain frozen for this milestone.

The old scripted play is allowed only as a training teacher/baseline. Imitation and
DAgger initialize manager decisions; alternating native PPO then updates all three
strategic/opponent roles. Report these contributions separately, without assuming
PPO improves playing strength. Preserve failed candidates and every test outcome.

## Scenarios and split protocol

The bounded drill starts with one blue robot holding the ball. Families are evenly
cycled by seed modulo3: midfield passing, advanced flank possession, close goal
possession. Both goal directions and blue IDs vary. Family labels/seed/outcome are
not policy observations. Both neural defenders stay active across every decision.
Midfield defense initially blocks the direct pass; advanced/close situations start
with the field defender behind the holder and keeper near goal. This tests action
choice across starting situations rather than full-match recovery or counterattack.

| Purpose | Seeds |
| --- | --- |
| Teacher development |3020000–3020059|
| Native demonstrations |3000000–3000299|
| Mixed DAgger |3010000–3010149|
| PPO resets only |3100000–3199999|
| Learned development |3200000–3200059|
| Broader development |3201000–3201119|
| Shared cross-play |3202000–3202029|
| Reserved final/final |3300000–3300239|
| Reserved final attack/bootstrap defense |3310000–3310119|
| Reserved bootstrap attack/final defense |3320000–3320119|
| Consecutive fresh demo |3400000–3400008|

Initial plan:300teacher and150mixedDAgger episodes, then three generations with
32,768PPOsteps each for defender, keeper and manager. Select the last completed
generation using only development results. Record all updates, hashes and opponent
pool choices. Development-only amendments must be documented before reserved tests.

Freeze and publish models/runtime before reserved cases. Never tune on those cases
or reuse them to qualify a changed candidate. Gates:≥95% clean outcomes (goal or
legitimate defensive contact stop),≤2% robot collisions,≤3% unforced errors; final
pair≥20% goals and final defense≥20% stops against bootstrap. Each final-pair family
must have≥90% clean outcomes. Also require≥20 actual passes,≥20 stable receptions,
≥20 episodes with≥0.5m controlled carry,≥20 actual shots and≥5 goals without a prior
pass in the240final cases. These measure exercised decisions, not tactical optimality.
All nine demo cases must be clean; include every case in original order. A failed
qualification/demo stays published; a new candidate requires new reserved seeds.

Audit physical kick events, stable measured reception, possession, collisions,
whole-ball goal crossing, both opponents' uninterrupted inference/physics counts,
manager decisions, target variation and model/runtime hashes. Test that inference
never calls scripted planners or derives behavior choice from measured reception.
Keep decisions/logs for losses and defensive wins, not only goals. Compare a teacher
baseline and all retained snapshot pairings on development scenarios.

Approximate ground-truth Webots simulation, ideal body velocities, spring dribbler,
velocity kicks, contact-distance/modeled mouth stop predicate and initial possession
remain explicit limits. No full-match, official-rule or hardware qualification.
