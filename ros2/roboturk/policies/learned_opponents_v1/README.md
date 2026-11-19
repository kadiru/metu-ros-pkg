# Continuous learned defender and goalkeeper v1

Frozen November19,2026 before reserved tests; native Webots R2025a
nightly17/11/2026. Both movement controllers are neural checkpoints. No analytic
movement fallback is present in deployment.

| Defensive challenge | Reserved result | Wilson95% interval |
|---|---:|---:|
| Incoming1.8m/s longitudinal pass: physical interception |100/100|96.30–100%|
| Incoming3.5m/s longitudinal shot: contact and no goal through2s |99/100|94.55–99.82%|

Defender seeds1250000–1250099; keeper1260000–1260099. The keeper conceded on
seed1260065; the complete record is retained. There were34,424 defender and39,780
keeper physics checks at5ms. [Audited outcomes](qualification-audit.json),
[freeze](freeze.json), [defender records](defender/qualification.json),
[keeper records](keeper/qualification.json).

On the separate60-case development challenges, stationary robots with dribblers
enabled achieved45/60 interceptions and35/60 saves. The analytic teachers achieved
60/60 and57/60; final learned actors achieved60/60 and59/60 on the same development
seeds. These are direct, reachable threats, not proof of complete-match defense.

## Contract and actual learned control

`roboturk.continuous_opponent.v1`:33 inputs, three continuous outputs,192×192 MLP.
Both translations and yaw rate come from the network. The shared decoder applies
role-specific speed limits, bounded acceleration, keeper command latency, field-to-
body rotation and fixed dribbler enable. The keeper and defender run continuously
through pass, reception and shooting, without a reset at the attacking handoff.

Inputs include observed robot/ball state, role anchor, ball-crossing and holder-ray
projections, patrol clock, physical parameters and previous command. These are
engineered geometric features. Role assignment, anchors, actuator limits and
style context remain fixed. This is a learned controller with explicit context,
not perception learning or unrestricted multi-agent self-play.

Defender speed0.25–0.55m/s. Keeper speed0.65–0.95m/s, acceleration2–3m/s²,
latency0.08–0.14s. Direct challenge starts1.8–2.8m before the assigned line with
initial lateral error≤0.12m. Keeper shot target y±0.28m; defender pass target within
±0.22m of its initial lane. Actual total ball speed includes the lateral component
and can exceed the stated longitudinal speed. Save success is not a possession-
retention claim. The earlier block/recover micro behavior has a different contract.

## Training and selection

Native full-play teacher collection:159/160 attacker goals; one intercepted pass.
Each opponent also received300 direct challenge demonstrations. Teacher defender:
300/300; teacher keeper:292/300, with eight conceded goals retained. After cloning,
80 mixed-DAgger full plays produced80 receptions and66 goals, all14 failures being
keeper saves. Each actor received another150 mixed direct challenges (defender150,
keeper147 successes). Training arrays remain in ignored local runs; all stage
checkpoints and outcome records are retained for publication.

| Actor | Cloning development | DAgger development | Anchored PPO development |
|---|---:|---:|---:|
| Defender |60/60|60/60|60/60|
| Keeper |59/60|57/60|59/60|

Each final actor completed32,768 PPO steps with32 accepted actor updates and zero
rejections. Defender PPO starts from its selected imitation stage; keeper returns
to the better cloned actor before PPO. Release selection uses defensive development
success count; final PPO breaks ties. Attacker win rates are not selection criteria.
There is no incremental PPO success-rate gain over the selected imitation actors.
One CPU training seed127 per actor; no independent training-seed replication.

| Actor | SHA256 |
|---|---|
| Defender |`06fd042a37390dd0e51ab73f668763730a2e52faade721fbd421fa847612a726`|
| Keeper |`9dea9369b2b6bfe3811ca338c4d89f9b7854f7146ae0f33f1fd755967ecc52f4`|

[Protocol and reproduction](../../docs/LEARNED_OPPONENTS.md),
[team-paper inspiration](../../docs/BEHAVIOR_SOURCES.md). The controllers extend
CMDragons-inspired lane blocking and ER-Force-inspired keeper prediction. Their
papers are not being described as using this RL implementation.

Ground-truth state, ideal body velocity, spring dribbler and applied velocity kicks
in approximate Webots physics; no calibrated omniwheel/roller model or hardware
validation. Full-play attacker qualification is a separate artifact and was pending
when these opponents were frozen.

The unchanged previous attacking play scored45/60 against these exact frozen
opponents on full-play development seeds1240000–1240059. All60 passes were
received; all15 failures were keeper saves, with zero pre-release possession
losses. [Complete frozen-opponent baseline](old-attackers-baseline.json).
