# Learned tactical decisions and self-play v2 — evaluation in progress

A neural manager chooses pass/receive, dribble or shoot, the acting robot,
receiving and dribble targets, receiving flank, goal corner and when to retain or
update a receiving point. The75-input/12-output v4 contract runs throughout the
play. Frozen learned motion policies execute its decisions; no scripted tactical
planner or reception-triggered handoff runs at inference. Both opponents retain
full planar neural control.

The candidate completed two alternating self-play generations:196,608native Webots
PPO steps and192accepted updates across manager, defender and keeper. The selected
candidate is the last completed generation, not a weaker opponent pairing. The
training audit verifies actor lineage, every reset/opponent-pool record, changed
actors and exact preservation of the manager's geometry regressor. Discrete
flank/behavior/owner/corner/update heads remain trainable.

| Development panel | Goals | Defensive stops | Clean |
|---|---:|---:|---:|
| Factored-flank initialization,60games |28|32|60/60|
| Initialization,120broader games |51|68|119/120|
| Generation1,60games |30|29|59/60|
| Generation2,60games |32|27|59/60|
| Generation2,120broader games |47|70|117/120|
| Generation2,240extended games |123|112|235/240|
| All9historical pairings,270games |157|113|270/270|

The final broader panel has two ball-outs and one collision; ten stable receptions.
The extended panel recorded38stable receptions and118carry episodes; failures were
three collisions, one robot field exit and one ball-out. Native portable-bundle and
recording preflights passed. **Reserved qualification and the fresh demo are pending.**
[Protocol and unchanged release gates](../../docs/LEARNED_DECISIONS.md),
[training audit](training-audit.json), [generation2 outcomes](artifacts/round2/development.json).

Initialization is explicit:750native teacher/mixedDAgger collection episodes,
176,011manager observations and780supervised manager epochs, including label/contract
conversions. Keeper boundary coaching and defender relative-speed coaching used
100supervised epochs each on native observations with derived labels. These are
training contributions, with no runtime action projection. V1's failed three-generation
league (294,912PPOsteps) is retained in [decision_self_play_v1](../decision_self_play_v1/README.md).
Its defensive ancestors were retained and coached; its manager PPO updates were
replaced by the earlier manager before v4 refitting. No isolated PPO improvement is
claimed. All failed preparation panels, checkpoints and data provenance remain.

Scope: an initial-possession two-attacker versus defender/keeper drill, both goal
directions and blue IDs, three starting families. Native ground-truth Webots state,
approximate body velocities, spring dribbler, velocity kicks and contact-stop
predicates remain explicit. This is not full-match, calibrated hardware or a live
ROS/Webots-graph qualification. ROS/core build checks are separate. Prior qualified
micro skills and paper provenance remain in the repository.
