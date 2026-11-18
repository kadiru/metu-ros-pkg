# Learned two-robot pass, receive and score

The released play scored **200/200 reserved Webots scenarios**, with zero premature kicks, possession losses before either kick, or robot collisions. A centralized 36-input/seven-action transfer network coordinates two teammates. After a real pass and 0.4 s of stable reception, a scripted play assigns the existing learned scoring policy to the receiver. There is no physics reset or teacher fallback at the handoff. This is hierarchical skill composition, not decentralized multi-agent RL or an end-to-end learned strategy. [Model card, checkpoints and complete records](../policies/pass_receive_v2/README.md).

The [native Webots demo](../media/pass-receive-score-demo.mp4) scored **6/6** on consecutive preselected fresh seeds 560000–560005. All outcomes are included in the 77.64-second recording at normal simulation speed.

## Scenario and acceptance target

These criteria were declared before qualification. The passer starts with a stationary ball at its dribbler; both robot headings span the full circle. Attack-relative passer x is −2.0 to −0.7 m and y is ±1.1 m. The receiver starts 1.3–2.4 m forward and ±0.9 m laterally. Both attacking directions and both robot-ID role assignments are sampled. Two teammates, an unguarded goal, ground-truth state, no opponents or obstacles.

Require 0.4 s stable control before an actual pass kick. Launch-to-teammate distance and ball displacement before stable reception must each be at least 1.0 m. The receiver must establish 0.4 s continuous mouth possession with relative speed below 0.2 m/s. Without resetting the physics, it must then carry at least 0.5 m closer to goal and score with an actual kick from within 1.8 m. The whole ball must cross the opponent goal line inside the posts.

Passer possession loss before release, receiver possession loss after stable reception and before shooting, robot collision, invalid kick, field exit or timeout fails the episode. Transfer timeout: 16 s; scoring timeout: 24 s. Possession and collision checks use 5 ms physics samples.

Acceptance target: at least 95% complete-sequence success over 200 reserved scenarios, with zero collisions and zero possession-loss episodes before either valid release. The checkpoint was frozen before qualification seeds 550000–550199, which were used once. Report transfer-only and complete-sequence outcomes separately.

## Policy and training

Observations comprise the passer's goal-conditioned ball state targeting its teammate (17), receiver ball state (12), relative heading cosine/sine (2), measured pass flag and possession dwell counters (3), and signed passer/receiver aiming errors divided by π (2). Actions command passer vx/vy/omega/fire and receiver vx/vy/omega. The skill interface retains the passer dribbler until firing, requests a fixed 1.8 m/s pass, holds the passer after its actual release, and enables the receiver dribbler. The receiver cannot kick until assigned the scoring skill, which uses a fixed 3.5 m/s shot. Independent dribbler duty, kick power and the role-handoff decision are not learned. Early pass requests are evaluated as failures, not silently blocked.

The original 34-input experiment used 300 Webots teacher demonstrations (400000–400299), 80 cloning epochs, 150 DAgger episodes (420000–420149), 50 refit epochs and 65,536 constrained PPO steps, seed 47. Development 410000–410059: cloned 54/60 receptions, DAgger 37/60, final PPO 56/60. All failures were pass-release timeouts, with no possession losses. All 64 actor updates were rejected; the evaluation difference does not establish a PPO actor benefit. V1 was not submitted to qualification or demo seeds. [All three checkpoints and raw records](../policies/pass_receive_v1/README.md).

V2 adds explicit aiming errors, starts from the v1 cloned checkpoint and reuses only its training observations. It refits for 80 epochs, collects 150 new DAgger episodes (520000–520149), then refits for 50 epochs. Mixed teacher/student collection received 148/150 passes; two premature-pass failures are retained. On new development seeds 510000–510059, both angle refit and DAgger passed 60/60. Select the warm start by success count, possession losses and mean successful duration; DAgger was selected.

Then run 65,536 constrained PPO steps, seed 57, one CPU thread, learning rate 3e−7, with reference-policy regression on training observations. Actor-change probe bounds: normalized-action MSE 1e−6 and maximum absolute difference 0.01. Projection learning rate: 3e−6. All 64 updates were accepted. Final transfer development: 60/60, mean 4.9453 s. Full learned-play development: 60/60 goals, mean 12.265 s. DAgger already passed development, so **no incremental PPO success benefit or learning-from-scratch claim is made**. These are sequential experiments, not independent training-seed replications. No development, qualification or demo observations were added to training.

The analytic transfer teacher composed with the learned scorer separately completed 40/40 full sequences. That validates the curriculum; it is not learned transfer-policy evidence.

## Reproduce

For exact v1 regeneration, use [the original source revision](https://github.com/kadiru/metu-ros-pkg/commit/0f95043714d4c2e4fa66175d1fe87efe94c2af38). Current `team-train` uses the 36-input contract. The original command generates the v1 checkpoint and training dataset:

```sh
../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode team-train --steps 65536 --seed 47 --output runs/team-v1 --timeout 1200
```

Then use current code:

```sh
../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode team-upgrade --checkpoint runs/team-v1/bc.zip \
  --steps 65536 --seed 57 --output runs/team-v2 --timeout 1200

../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode team-evaluate --checkpoint runs/team-v2/policy.zip \
  --shoot-checkpoint policies/capture_dribble_shoot_v2/policy.zip \
  --episodes 200 --seed 550000 --output runs/team-v2/qualification --timeout 600

../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode team-demo --checkpoint runs/team-v2/policy.zip \
  --shoot-checkpoint policies/capture_dribble_shoot_v2/policy.zip \
  --episodes 6 --seed 560000 --output runs/team-v2/demo --timeout 240 --render
```

## Deployment and limits

`PassReceiveScorePlay` is the core deployment API. Assign distinct teammate roles and supply world state, measured physics substeps and actual kick feedback. The play rejects time rewinds or duplicate frames, and holds on referee stops or missing perception. Full learned evaluation and demo invoke it directly and assert that its memory agrees with an independent task monitor. The current ROS message graph does not yet provide the kick feedback this play requires; full live ROS integration remains future work.

Dynamics are approximate: ideal body velocity, ODE ball contacts, spring/damper dribbler and edge-triggered kicks. No calibrated hardware, opponents, goalkeeper, perception noise, radio latency or full SSL rules. Arbitrary initial-ball acquisition by the pair, passes to running teammates, variable kick power and full-team match strategy remain unqualified.

API references: [Webots Supervisor](https://github.com/cyberbotics/webots/blob/master/docs/reference/supervisor.md), [Stable-Baselines3 PPO](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html).
