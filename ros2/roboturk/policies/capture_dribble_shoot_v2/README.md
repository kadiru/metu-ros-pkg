# Capture, dribble and score policy v2

Qualified November 18, 2026 (America/Los_Angeles). A single stateless neural policy captures an unpossessed ball, dribbles toward the opponent's goal, aims and requests a shot. Its 17th input explicitly encodes goal distance. There is no analytic motion fallback, reference network or scripted phase selector during inference.

| Reserved qualification measurement | Result |
| --- | --- |
| Full capture → retained carry → kick → goal sequence | **200/200**, seeds 350000–350199 |
| Premature/invalid shots | **0/200** |
| Possession-loss episodes after acquisition and before release | **0/200** |
| Physics possession checks before release | **241,996**, every 5 ms |
| Stationary / initially rolling balls | **112/112 / 88/88** |
| Positive / negative attacking goal | **106/106 / 94/94** |
| Mean full-sequence duration | **8.3683 s** |
| Mean controlled progress toward goal before shooting | **3.1708 m** |
| Mean actual shot distance | **1.2865 m** |
| Wilson 95% success interval | **98.12–100%**, for this scenario distribution |
| Fresh preselected demo, seeds 360000–360005 | **6/6 full-sequence goals** |

The target was at least 95% empirical full-task success and no pre-shot possession losses on 200 reserved trials. The checkpoint was frozen before this qualification and not tuned afterward. Every successful episode had exactly one applied kick. Controlled progress ranged 1.296–4.933 m; the largest measured goal-line crossing offset from goal center was 76.72 mm.

## What was tested

One robot, an unguarded goal, no obstacles, ground-truth state. Initial ball x ±1.7 m/y ±0.9 m; robot-ball distance 0.35–2 m with arbitrary heading and bearing. Ball starts stationary or rolling at 0.15–0.6 m/s. Both attacking directions are sampled, with the actual opponent goal center at (±4.5 m, 0) as the target.

Acquisition requires 0.4 s of continuous mouth possession, active dribbler and relative speed below 0.2 m/s. Then the ball must move at least 0.5 m closer to goal under continuous possession. Every 5 ms physics sample is checked. A valid release requires an actual backend kick from possession within 1.8 m of goal. Scoring requires the **whole ball to cross the opponent goal line inside the posts**, interpolated between physics samples. A pushed-in ball, premature shot, own goal, missed goal, field exit or 24 s timeout cannot succeed.

The network learns vx/vy/angular velocity and fire intent. The skill interface keeps the dribbler on until firing, then switches it off and requests a fixed **3.5 m/s** simulated kick. Independent dribbler duty and kick power are not learned. No goal-distance rule silently blocks bad network fire requests; they fail evaluation.

## Training and experiment history

The [v1 experiment](../capture_dribble_shoot_v1/README.md) scored 192/200 and 5/6 in its preselected demo, exposing premature shots. Its complete records and video are retained. V2 uses a new input contract, new recovery/development seeds, a new reserved qualification set and new preselected demo seeds. No v1 qualification or demo observations were added to training.

| V2 development stage, seeds 310000–310059 | Goals | Possession losses | Invalid shots |
| --- | ---: | ---: | ---: |
| Range feature and refit, before new recovery data | 0/60 | 0 | 0 |
| After 150 new DAgger episodes and refit | 60/60 | 0 | 0 |
| Released anchored PPO, 65,536 steps | 60/60 | 0 | 0 |

The initial range refit carried past the shooting point without firing; new recovery examples taught the braking/shot transition. The actor starts from v1's DAgger checkpoint, reuses its training observations with the new range column, and adds 150 DAgger episodes (320000–320149). Refitting uses 80 then 50 epochs. Constrained PPO uses seed 37, one CPU thread, learning rate 3e−7 and training-observation reference-policy regression; all 64 actor updates satisfied the probe bounds. The original pretraining seed was 27. These are sequential experiments, not independent training-seed replications.

DAgger already passed all development episodes before PPO; no incremental PPO success benefit is established. Full method, fixed actuator semantics and reproduction steps are in [SCORING_TRAINING.md](../../docs/SCORING_TRAINING.md).

## Artifacts and integration

- [policy.zip](policy.zip) and required [policy.json](policy.json). SHA-256: `6558b977dcdeb0001e9590b01ca1cfb9f3ec3fd16da1f43d17b8c3e6c9f6cc11`.
- [qualification.json](qualification.json): every reserved outcome, applied-kick count, carry progress and actual goal-crossing coordinate. [development.json](development.json), [bc-development.json](bc-development.json) and [dagger-development.json](dagger-development.json) preserve all development outcomes, including the initial 0/60 result.
- [experiments.json](experiments.json): all stage hashes/manifests/aggregates. [anchor-updates.json](anchor-updates.json): actor-update diagnostics.
- [Native Webots demo](../../media/capture-dribble-score-demo.mp4) and [demo.json](demo.json): consecutive preselected fresh seeds 360000–360005, every outcome retained, normal simulation speed, actual physics and route/tracking views. No generated animation or teacher fallback.
- Training source: local `a42628c`, published [`5edfd064`](https://github.com/kadiru/metu-ros-pkg/commit/5edfd064b7dde041e55d3bb7d812754199b0bff9). Later documentation/preview changes do not modify the policy or physics.
- [Runtime provenance](../../docs/runtime-manifest.json): Webots R2025a nightly 17/11/2026, commit `e5d1122a3cd59f007ee4b37c8f651b50e7ee83d2`. [Python environment](../../requirements-macos-validation.txt).

Load with `CheckpointPolicy(path, Skill.SHOOT, Config())` and register through `BehaviorLibrary` or the ROS strategy node's `shoot` policy parameter. `Intent.target` is the opponent goal center. The shared training/inference contract is `roboturk.capture_dribble_shoot.v2` (17 observations, four actions); older v1 checkpoints remain loadable using their 16-input contract.

## Limits

Approximate ideal body velocity, ODE contacts, spring/damper dribbler and edge-triggered ball-velocity kick. No calibrated wheels/rollers/kicker, goalkeeper, opponents, vision noise, radio latency, full SSL rules, arbitrary shooting targets, full-team match benchmark or hardware transfer. The ROS checkpoint interface exists, but this evidence is a direct Supervisor skill benchmark, not a complete Webots/ROS game run.
