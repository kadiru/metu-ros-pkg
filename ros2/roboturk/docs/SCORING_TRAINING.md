# Learned capture, dribble and score

## Experiment progression

The released **v2 scored 200/200** on its new reserved seeds 350000–350199, with zero invalid shots or pre-shot possession losses over 241,996 physics checks. Mean complete-sequence time was 8.3683 s, controlled progress 3.1708 m and shot distance 1.2865 m. [Released checkpoint, model card and full records](../policies/capture_dribble_shoot_v2/README.md).

The original 16-input v1 scored 192/200 reserved episodes and 5/6 preselected demo episodes. It met the numerical acceptance target but retained premature-shot defects, so v2 is a separate follow-up experiment. V2 adds explicit radial goal distance as a 17th input, with contract `roboturk.capture_dribble_shoot.v2`. The action semantics and success criteria are unchanged. V1 remains loadable under its original contract.

V2 reuses only v1's training demonstrations/DAgger observations and the pre-PPO DAgger checkpoint. It adds the range column, transfers the original network with a zero-initialized extra input weight, refits for 80 epochs, collects 150 new DAgger episodes (320000–320149), refits for 50 epochs, and chooses the warm start on new development seeds 310000–310059. Constrained PPO then runs for 65,536 steps with seed 37. New qualification seeds are **350000–350199** and new preselected demo seeds **360000–360005**. No v1 qualification/demo observations are added to training. All v1 outcomes are preserved in its model folder.

To reproduce v2, first regenerate the v1 checkpoint/dataset at [the original source revision](https://github.com/kadiru/metu-ros-pkg/commit/2aaaba583b98d15a7bce8fc2a2a748dd2b489bd4), then run the upgrade on current code:

```sh
../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode shoot-upgrade --checkpoint runs/shoot-v1/dagger.zip \
  --steps 65536 --seed 37 --output runs/shoot-v2 --timeout 900

../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode shoot-evaluate --checkpoint runs/shoot-v2/policy.zip \
  --episodes 200 --seed 350000 --output runs/shoot-v2/qualification --timeout 300

../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode shoot-demo --checkpoint runs/shoot-v2/policy.zip \
  --episodes 6 --seed 360000 --output runs/shoot-v2/demo --timeout 240 --render
```

The v1 procedure and its original seed ranges below document that earlier experiment. Current `shoot-train` uses the 17-input contract; use the linked historical revision for exact v1 regeneration.

`capture_dribble_shoot.randomized.v1` starts with an unpossessed ball and requires the complete acquisition → retained carry → kick → goal sequence. The goal is unguarded. This is a skill benchmark in approximate Webots dynamics, not competitive SSL play.

## Task and predeclared acceptance target

- Initial ball x ±1.7 m, y ±0.9 m; robot-ball distance 0.35–2 m; full-circle robot heading and bearing. Half the scenarios sample an initially rolling ball at 0.15–0.6 m/s; otherwise it starts stationary. Both attacking directions are sampled.
- Acquisition requires 0.4 s of continuous mouth possession, active dribbler and relative speed below 0.2 m/s. After acquisition, any possession loss at a 5 ms physics sample before the valid shot fails the episode.
- Before shooting, decrease ball-to-goal distance by **at least 0.5 m after acquisition**. A valid shot requires an actual backend kick event from possession, within 1.8 m of goal. Premature firing cannot skip acquisition or carrying.
- Score only when the **whole ball crosses the opponent's goal line inside the posts**, detected by interpolating consecutive physics samples. Crossing without a valid shot, an own goal, crossing outside the posts, leaving the field or reaching the 24 s timeout fails.
- Require **at least 95% complete-sequence success on 200 reserved scenarios and zero pre-shot possession-loss episodes**. Development: 210000–210059. Demonstrations: 200000–200299. DAgger: 220000–220149. Qualification: 250000–250199. Demo: six consecutive fresh seeds 260000–260005; retain every outcome.

## Policy and training

The earlier frozen v1 candidate achieved **192/200 goals (96%)**, with no post-acquisition possession losses. Eight episodes failed through premature firing before acquisition completed. Mean successful duration was 9.7573 s, with 3.0098 m of controlled progress and 1.5520 m shot distance. Its Wilson 95% interval is 92.31–97.96%; meeting the empirical target does not establish a true success probability above 95% with that confidence. [Earlier checkpoint and complete outcome records](../policies/capture_dribble_shoot_v1/README.md).

One stateless MLP receives the existing 16-dimensional body-relative ball/robot/target observation, with the target now being the actual opponent goal center (±4.5, 0). Its four actions are body vx/vy, angular velocity and a binary fire request represented by the sign of the fourth action. The skill interface keeps the dribbler on while fire is false; on fire it disables the dribbler and requests a fixed **3.5 m/s** kick. Kick power and independent dribbler duty are not learned in this version. There is no scripted phase selector, analytic fallback, teacher or reference network at inference. Shared contract: `roboturk.capture_dribble_shoot.v1`, registered as `Skill.SHOOT`.

Initial experiment: 300 actual Webots demonstrations and 80 balanced actor-cloning epochs, followed by 150 DAgger episodes (80% student execution, 20% teacher), 50 refit epochs, and development evaluation of both checkpoints. Choose the warm start by development successes, possession losses and mean successful duration, in that order. Qualification remains untouched.

Then run 65,536 constrained PPO steps, seed 27, one CPU thread. Actor/critic each have two 128-unit layers; initial log standard deviation −4, learning rate 3e−7, gamma 0.995, 1,024 rollout steps, batch 256, five PPO epochs, clipping 0.1, entropy bonus zero. Eight reference-policy projection steps follow each PPO update. Acquisition, carrying and shooting observations are balanced. A fixed 12,288-observation training probe rejects actor updates with normalized action MSE >1e−6, maximum absolute difference >0.01 or nonfinite changes. The critic can continue learning. Probe bounds do not guarantee behavior on unseen states; the frozen checkpoint must still qualify.

Report the cloned, DAgger and final development results separately. Do not infer a PPO benefit if imitation already succeeds. A single training seed is not evidence of training robustness.

## Reproduce

```sh
../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode shoot-train --steps 65536 --seed 27 \
  --output runs/shoot-v1 --timeout 1200

../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode shoot-evaluate --checkpoint runs/shoot-v1/policy.zip \
  --episodes 200 --seed 250000 --output runs/shoot-v1/qualification --timeout 300

../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode shoot-demo --checkpoint runs/shoot-v1/policy.zip \
  --episodes 6 --seed 260000 --output runs/shoot-v1/demo --timeout 240 --render
```

The simulator uses ideal body velocity, ODE ball contacts, a bounded spring/damper dribbler and an edge-triggered ball-velocity kick. Motor/roller/kicker calibration, perception noise, latency, opponents, goalkeepers, arbitrary shooting targets, complete rules and hardware transfer remain outside this qualification.

API references: [Webots Supervisor](https://github.com/cyberbotics/webots/blob/master/docs/reference/supervisor.md), [SB3 PPO](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html), [policy architectures](https://stable-baselines3.readthedocs.io/en/master/guide/custom_policy.html).
