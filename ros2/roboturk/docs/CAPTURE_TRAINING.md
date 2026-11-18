# Learned chase and capture

This task trains a neural motion/dribbler policy in Webots. It uses an analytic controller to supply demonstration labels, then DAgger recovery examples and PPO reinforcement learning. The checkpoint's inference path contains no teacher, blending, scripted pursuit or fallback.

## Task and acceptance target

One robot, unobstructed field, ground-truth state. Robot-ball starting distance is 0.35–2.0 m; robot heading and relative bearing cover the full circle. Ball starts within x ±1.7 m and y ±0.9 m. Robot position is sampled inside x ±3.7 m and y ±2.3 m. Half the scenarios have a stationary ball; the others start with 0.15–0.6 m/s ball speed and random travel direction. Webots contact/friction dynamics then determine the ball's trajectory.

Success requires the ball to remain within the robot mouth region, robot-ball relative speed below 0.2 m/s, and dribbler output above 0.5 for **20 consecutive control frames (0.4 seconds)**. Leaving the field fails; the time limit is ten seconds. Touching the ball briefly is insufficient.

The predeclared qualification target is at least 95% empirical success on 200 unseen Webots episodes. Report stationary/moving subsets, capture time and Wilson confidence interval. This measures the stated distribution, not all possible soccer situations.

The final checkpoint passed **200/200**, split into 100/100 stationary and 100/100 initially moving balls. Mean capture time was **1.628 s**; the Wilson 95% interval was **98.12–100%**. Six further consecutive demo episodes all succeeded and maintained capture for an additional second. [Checkpoint, model card and raw records](../policies/chase_capture_v1/README.md).

## Training method

- Collect 300 teacher episodes in actual Webots, seeds 0–299.
- Fit the neural actor to the demonstrated commands (60 epochs).
- Collect 150 DAgger episodes, seeds 20000–20149: execute the student 80% of steps and teacher 20%, always labeling with the teacher.
- Refit the actor to the combined data (40 epochs).
- Fine-tune with PPO in actual Webots: 131,072 steps, seed 7, one CPU thread. Actor/critic each use two 128-unit layers, initial log standard deviation −3, 1,024 rollout steps, batch 256, five PPO epochs, learning rate 1e−5, clipping 0.1, gamma 0.99.
- Development seeds 10000–10049. Qualification seeds 50000–50199 are reserved and are not used to select a checkpoint. Demo seeds 60000–60005 are another fresh set, recorded consecutively without removing failures.

Both the initial cloned actor and final PPO policy passed 50/50 development episodes. The result demonstrates a working learned policy with PPO fine-tuning; it does not isolate PPO's contribution. Only one training seed was run.

The body-relative `roboturk.capture.v1` observation contains 12 floats: relative ball position, unit bearing, ball and robot planar velocities, robot angular velocity, magnified mouth error, and possession flag. Actions are body vx/vy, angular velocity and dribbler duty. Kicking is unavailable. The same encoder/decoder is used by the Gym environment and `CheckpointPolicy` inside the behavior library/ROS node.

## Reproduce

Use the project venv and the verified Webots runtime described in the main README:

```sh
../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode capture-train --steps 131072 --seed 7 \
  --output runs/capture-v1 --timeout 1200

../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode capture-evaluate --checkpoint runs/capture-v1/policy.zip \
  --episodes 200 --seed 50000 --output runs/capture-v1/qualification --timeout 300

../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode capture-demo --checkpoint runs/capture-v1/policy.zip \
  --episodes 6 --seed 60000 --output runs/capture-v1/demo --timeout 180 --render
```

Set `ROBOTURK_DEMOS` or `ROBOTURK_DAGGER` to change collection budgets for a new experiment. Changing a budget creates a different experiment and must be recorded with its results. Checkpoints and large intermediate arrays are ignored in `runs/`; a qualified release is copied explicitly into `policies/` with provenance and evaluation.

## Scope

The Webots body-velocity actuator and spring/damper dribbler are approximate. This policy does not handle opponents, obstacle avoidance, perception noise, radio latency or physical robots. A calibrated wheel/roller/dribbler model and additional qualification are required for those claims. Both the teacher and learned policy act on the same physics backend.
