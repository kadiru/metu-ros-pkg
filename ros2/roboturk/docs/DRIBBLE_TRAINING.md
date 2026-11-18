# Capture and dribble toward the opponent's goal

The `capture_dribble.randomized.v1` task starts with an unpossessed ball and requires acquisition, continuous carrying toward the opponent's goal, and a stationary one-second hold within 0.18 m of the target. The target is (±3.6 m, 0), 0.9 m in front of the selected goal line. Both attack directions are sampled. There is no kick action.

Initial ball positions, rolling speeds, robot distances and headings use the capture curriculum: ball x ±1.7 m / y ±0.9 m; robot-ball distance 0.35–2 m; full-circle bearing and robot heading; 50% stationary balls and otherwise initial ball speed 0.15–0.6 m/s. Timeout is 24 seconds. Opponents and obstacles are absent.

Acquisition requires 0.4 s of continuous mouth possession, active dribbler and relative speed below 0.2 m/s. After acquisition, **any loss of mouth possession or dribbler activation fails immediately**, including losses between control frames: Webots snapshots are checked every 5 ms (200 Hz). At the destination, both robot and ball speeds must remain below 0.15 m/s for 50 control frames. Stopping near the goal without the ball cannot succeed.

## Acceptance target and data separation

Before qualification: require at least 95% full-task success on **200 reserved episodes**, and **zero post-acquisition possession-loss episodes**. Qualification seeds are 150000–150199. Development uses 110000–110059; demonstrations use 100000–100299; DAgger uses 120000–120149. Fresh demo seeds are 160000–160005, recorded consecutively without removing failures. Report all attempts and subgroup counts, including both goal directions.

## Training

A single goal-conditioned 16-input/four-action neural policy controls acquisition, turning, carrying and stopping. The first 12 inputs are the existing capture observation; four additional inputs encode target displacement and unit direction in the robot frame. The shared `roboturk.capture_dribble.v1` contract is used by both Gym and the behavior/ROS checkpoint plugin. Inference does not call a teacher or blend scripted commands.

The analytic demonstration controller turns about the ball slowly, then carries it toward the target. The neural actor is trained on 300 Webots demonstrations, 80 balanced cloning epochs, 150 DAgger episodes (80% student / 20% teacher execution), and 50 refit epochs. Cloning balances acquisition and possession states. PPO then runs for 131,072 steps, training seed 17, one CPU thread. Actor and critic each have two 128-unit layers, initial log standard deviation −4, learning rate 3e−6, gamma 0.995, 1,024 rollout steps, batch 256, five PPO epochs, clipping 0.1 and no entropy bonus. Development evaluations before/after PPO distinguish successful imitation from additional RL benefit.

```sh
../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode dribble-train --steps 131072 --seed 17 \
  --output runs/dribble-v1 --timeout 1200

../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode dribble-evaluate --checkpoint runs/dribble-v1/policy.zip \
  --episodes 200 --seed 150000 --output runs/dribble-v1/qualification --timeout 300

../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode dribble-demo --checkpoint runs/dribble-v1/policy.zip \
  --episodes 6 --seed 160000 --output runs/dribble-v1/demo --timeout 180 --render
```

This remains the approximate Webots model: ideal body velocity, ODE contacts and a bounded spring/damper dribbler. It does not establish calibrated wheel/roller behavior, hardware transfer, robustness to noisy perception or contested possession. The prior capture policy remains available separately.

API references: [Webots Supervisor forces and movie recorder](https://github.com/cyberbotics/webots/blob/master/docs/reference/supervisor.md), [SB3 PPO](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html), [SB3 policy architectures](https://stable-baselines3.readthedocs.io/en/master/guide/custom_policy.html).
