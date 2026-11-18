# Capture-and-dribble policy v1

Trained and qualified November 18, 2026 (America/Los_Angeles). One goal-conditioned neural policy acquires the ball, carries it toward the opponent's goal and stops while retaining possession. It controls body vx/vy, angular velocity and dribbler duty; kicking is unavailable. Inference has no analytic fallback, teacher, reference policy or scripted phase controller.

| Reserved qualification measurement | Result |
| --- | --- |
| Complete capture, carry and stationary hold | **200/200**, seeds 150000–150199 |
| Post-acquisition possession-loss episodes | **0/200** |
| Physics-step possession checks | **438,456**, sampled every 5 ms |
| Stationary / initially rolling balls | **85/85 / 115/115** |
| Positive / negative attacking goal | **97/97 / 103/103** |
| Mean progress toward target after acquisition | **3.5867 m** |
| Mean full-task time | **13.0177 s**, including the one-second arrival hold |
| Wilson 95% success interval | **98.12–100%**, conditional on this distribution |
| Maximum measured mouth-position error after acquisition | **15.89 mm** |
| Fresh demo seeds 160000–160005 | **6/6**, all with a further 0.8 s of retained possession |

The acceptance target was ≥95% full-task success and zero post-acquisition possession-loss episodes on 200 reserved scenarios. The final checkpoint was frozen before qualification, and no held-out episode was used to select or adjust it.

## What success means

The unpossessed ball starts at x ±1.7 m, y ±0.9 m; robot-ball distance is 0.35–2 m, with full-circle headings and bearings. Half the scenarios sample initial ball speed 0.15–0.6 m/s; the rest are stationary. Both attacking directions are sampled. The target is (±3.6 m, 0), **0.9 m in front of the goal line**, so this is controlled approach rather than shooting/scoring.

Acquisition requires 0.4 s of mouth possession with dribbler activation and relative speed below 0.2 m/s. Thereafter, any missing mouth possession or dribbler activation at any 5 ms physics sample fails the episode immediately. Arrival requires the ball within 0.18 m of the target and both robot and ball speeds below 0.15 m/s for one second. The timeout is 24 s. Progress is the decrease in ball-to-target distance after acquisition, not total path length; the qualified range was 0.830–5.691 m.

## Training and rejected candidates

| Development stage, seeds 110000–110059 | Successes | Possession losses | Mean successful duration |
| --- | ---: | ---: | ---: |
| Cloned actor: 300 demonstrations, 80 epochs | 60/60 | 0 | 12.23 s |
| DAgger refit: 150 episodes, 50 epochs | 59/60 | 0 | 14.01 s |
| Unconstrained PPO: 131,072 steps, seed 17 | **0/60 — rejected** | 60 | — |
| Released constrained PPO: 65,536 steps, seed 18 | **60/60** | 0 | 13.03 s |

Unconstrained PPO lost possession while braking near the destination. The released policy restarts from the successful pre-DAgger cloned actor and alternates PPO updates with reference-policy regression on training observations. Eight projection steps follow each update; a fixed 12,288-observation training probe rejects an actor update if normalized action MSE exceeds 1e−6 or maximum difference exceeds 0.01. Two of 64 actor updates were rejected; the critic continued learning. The reference is used only during training.

This experiment establishes a working learned behavior and a way to preserve it during PPO fine-tuning. It **does not establish an incremental benefit from PPO**: the cloned actor already passed development and was faster there. One initialization seed and one refinement seed were tested, not independent repeated training runs. All development-stage records, including the failure, are included. [Exact training procedure and reproduction](../../docs/DRIBBLE_TRAINING.md).

## Artifacts

- [policy.zip](policy.zip) and required companion [policy.json](policy.json); shared contract `roboturk.capture_dribble.v1`, 16 observations and four actions. SHA-256: `1a6a6741f115093e54ad1d81fb25fcdf5f8583130d87ffb105b2e964dbb8779d`.
- [qualification.json](qualification.json): every reserved episode and aggregate metrics. [development.json](development.json): final candidate development.
- [experiments.json](experiments.json): checkpoint hashes, manifests and development aggregates for all four stages. Separate cloned, DAgger and rejected-PPO episode files preserve every outcome.
- [anchor-updates.json](anchor-updates.json): accepted/rejected actor update diagnostics. Constraints apply to a finite training probe; held-out performance still requires evaluation.
- [Native Webots demo](../../media/capture-dribble-demo.mp4) and [demo.json](demo.json): six consecutive fresh seeds 160000–160005, alternating route/tracking cameras. Normal simulation speed; no failures removed or synthetic animation substituted. The cyan square marks the goal-side arrival target and has no collision object.
- The same six seeds were recorded a second time to improve camera framing; every episode metric was identical. [Original camera-run outcomes](demo-original-camera.json) are retained. These are six distinct scenarios, not twelve independent tests. The published video is 81.96 s, 1280×720 at 25 fps.
- Training source: local `0993436`, published [`10089703`](https://github.com/kadiru/metu-ros-pkg/commit/10089703f7fbe9d299bb10b14be7e8e643c46df8). Later camera/presentation edits do not change physics or the checkpoint.
- [Webots runtime](../../docs/runtime-manifest.json): R2025a nightly 17/11/2026, commit `e5d1122a3cd59f007ee4b37c8f651b50e7ee83d2`. [Python environment](../../requirements-macos-validation.txt). All training used one CPU thread.

Load with `CheckpointPolicy(path, Skill.DRIBBLE, Config())`; register it for `Skill.DRIBBLE` in `BehaviorLibrary` or provide the `dribble` key in the ROS strategy node's `policies` JSON parameter. The `Intent.target` feeds the observation. Qualification covers the documented goal-side targets; arbitrary strategy targets and full-team play have not been qualified.

## Scope

One robot, unobstructed field, ground-truth observations, ideal body velocity, ODE contacts and a bounded spring/damper dribbler. No calibrated wheels/rollers, observation noise, latency, opponents, contested possession, full SSL rules or hardware transfer. The approximate model's result is not evidence of physical-robot reliability.
