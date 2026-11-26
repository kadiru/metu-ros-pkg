# Capture, dribble and score: v1 experiment

This earlier candidate is retained as an experiment record. Its reserved score was 96%, but the six preselected demo episodes scored **5/6**. A premature-shot failure remained visible in the demo, motivating a separate v2 experiment with explicit goal-range input and new development/qualification/demo seeds. No v1 failure has been filtered or relabeled.

Trained and evaluated November 18, 2026 (America/Los_Angeles). One stateless neural policy controls acquisition, carrying, aiming and the fire request. The skill interface retains the ball until firing and requests a fixed 3.5 m/s simulated kick. Inference has no teacher, analytic motion fallback, reference policy or scripted phase selector.

| Reserved qualification measurement | Result |
| --- | --- |
| Complete capture → carry → kick → goal sequence | **192/200 (96%)**, seeds 250000–250199 |
| Failures | **8 premature shots before acquisition completed** |
| Possession-loss episodes after acquisition, before shot | **0/200** |
| Post-acquisition physics possession checks | **273,852**, sampled every 5 ms |
| Stationary / initially rolling ball successes | **96/98 / 96/102** |
| Positive / negative attacking goal successes | **90/94 / 102/106** |
| Mean successful sequence duration | **9.7573 s** |
| Mean controlled progress before shooting, successful episodes | **3.0098 m** |
| Mean actual shot distance | **1.5520 m** |
| Wilson 95% success interval | **92.31–97.96%**, conditional on this distribution |

This meets the predeclared **≥95% empirical success and zero pre-shot possession-loss** target. It does **not** establish that the true success probability exceeds 95% with 95% confidence. Premature firing remains a known defect, especially with initially moving balls. All eight failures are present in [qualification.json](qualification.json). The checkpoint was frozen before this reserved evaluation and was not adjusted afterward.

## Task definition

One robot, unguarded goal, no obstacles. Initial ball x ±1.7 m, y ±0.9 m; robot-ball distance 0.35–2 m, full-circle heading and bearing; stationary or initially rolling ball at 0.15–0.6 m/s. Both attacking directions are sampled. The policy's target is the actual goal center at (±4.5 m, 0).

Acquisition requires 0.4 s of continuous mouth possession with active dribbler and relative speed below 0.2 m/s. Then the ball must move at least 0.5 m closer to goal while retaining possession. A valid shot requires an actual backend kick event from possession, within 1.8 m of goal. Any premature shot fails; there is no runtime rule that silently suppresses bad network fire requests.

A goal requires the **whole ball to cross the opponent's goal line inside the posts**, using interpolation between consecutive 5 ms physics samples. Crossing without a valid shot, an own goal, crossing outside the posts, leaving the field or the 24 s timeout cannot succeed. Intentional release after the valid shot is distinguished from losing the ball before it.

All 192 successful episodes had exactly one applied kick. Controlled progress ranged 0.666–5.060 m; actual shot distances were 1.547–1.557 m. The largest goal-line crossing offset from goal center was 49.51 mm. These narrow shot ranges are a consequence of the demonstrated approach, not broad shooting-position qualification.

## Learning evidence

| Development stage, seeds 210000–210059 | Goals | Pre-shot possession losses | Premature shots |
| --- | ---: | ---: | ---: |
| Cloning: 300 demonstrations, 80 epochs | 55/60 | 0 | 5 |
| DAgger: 150 episodes, 50 refit epochs | 59/60 | 0 | 1 |
| Released anchored PPO: 65,536 steps | 59/60 | 0 | 1 |

DAgger improved the development success count. PPO retained that count; this experiment does not isolate an incremental PPO benefit. A single training seed (27) was used, with one CPU thread. PPO actor updates were constrained by reference-policy regression and a fixed training-observation probe; all 64 updates passed the declared bounds. No test seeds were used in training, warm-start selection or those constraints. [Exact method, split definitions and reproduction commands](../../docs/SCORING_TRAINING.md).

The 16 input floats encode body-relative robot/ball motion and goal displacement/direction. Four outputs control vx/vy/angular velocity and fire intent. The dribbler-on-until-fire behavior and fixed kick power are **skill actuator semantics**, not separately learned controls. Shared contract: `roboturk.capture_dribble_shoot.v1`, registered under `Skill.SHOOT`.

## Artifacts and provenance

- [policy.zip](policy.zip) and required companion [policy.json](policy.json). SHA-256: `c6a95030f8b06c491a75c101d252c8ef0ae5a241a7fd4da7e64b509c06f488f5`.
- [qualification.json](qualification.json): all 200 outcomes, including failures, with actual shot and goal-crossing metrics. [development.json](development.json), [bc-development.json](bc-development.json), [dagger-development.json](dagger-development.json): all development stages.
- [experiments.json](experiments.json): checkpoint hashes, manifests and aggregates. [anchor-updates.json](anchor-updates.json): actor-update diagnostics. [teacher-development.json](teacher-development.json): analytic comparison, kept distinct from neural results.
- [Native Webots demo](../../media/capture-dribble-score-v1-demo.mp4) and [demo.json](demo.json): **5/6 goals**, seeds 260000–260005. Seed 260004 failed with an invalid shot before acquisition completed. All outcomes are retained, at normal simulation speed, with route/tracking cameras and actual goal-line results. No synthetic animation or analytic fallback.
- Training source: local `a8ace63`, published [`2aaaba58`](https://github.com/kadiru/metu-ros-pkg/commit/2aaaba583b98d15a7bce8fc2a2a748dd2b489bd4). Later documentation/presentation edits do not alter the trained checkpoint.
- [Runtime provenance](../../docs/runtime-manifest.json): Webots R2025a nightly 17/11/2026, commit `e5d1122a3cd59f007ee4b37c8f651b50e7ee83d2`. [Python environment](../../requirements-macos-validation.txt).

Load using `CheckpointPolicy(path, Skill.SHOOT, Config())`, with `Intent.target` set to the opponent goal center, or the `shoot` entry in the ROS strategy node's `policies` parameter. The full ROS strategy graph and arbitrary mid-play invocation are not qualified by these single-skill episodes.

## Limits and next work

Approximate ideal body velocity, ODE contacts, spring/damper dribbler and edge-triggered ball-velocity kick. No calibrated motors/rollers/kicker, independently learned kick strength, noisy vision, radio latency, opponents, goalkeeper, complete SSL rules or hardware transfer. The first improvement should target premature firing with new training data and a new reserved test set; do not tune on this release's qualification seeds and present them as fresh evidence.
