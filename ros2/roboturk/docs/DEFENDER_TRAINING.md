# Passing around a reactive lane defender

This extends the two-attacker play with an independently controlled yellow defender. The new actor observes the defender and can move both attackers before deciding to pass. The previous learned scorer remains frozen and is assigned to the receiver after stable reception. There is no physics reset at handoff and no analytic attacker fallback at checkpoint inference.

## Opponent and scenario

The defender guards its initial longitudinal line. Before a pass it patrols ±0.07 m around the initially blocked lane, with randomized period 2–4 s and phase. When a rolling ball is predicted to cross its line in 0–2.5 s, it moves toward that crossing point and faces the ball with its dribbler enabled. Speed limit is sampled uniformly from 0.25–0.55 m/s. This is a fixed analytic lane guard, not a free-ranging adversary, goalkeeper or self-play opponent. It receives commands and is checked at every physics step through transfer and scoring.

The passer starts holding a stationary ball. Attack-relative x is −2.0 to −1.2 m; y is ±0.55 m. The receiver starts 1.8–2.3 m forward and 0.30–0.55 m to either side. Both attackers' headings span the full circle; robot roles and attacking direction vary. The defender begins 50–60% along the ball-to-receiver segment with at most 0.025 m lateral perturbation, blocking the initial straight pass.

The attacker's demonstrations move the receiver wider (2.2 m laterally from the ball), align the passer, release a fixed 1.8 m/s pass, and receive it. The learned policy is free to command different actions; no lane-clearance rule silently blocks a bad fire request. The flank route is a demonstration target, not a scripted waypoint controller at inference.

## Acceptance declared before qualification

Preserve the previous pass, reception and scoring criteria: 0.4 s stable passer possession; an actual pass over at least 1 m; 0.4 s stable receiver possession with relative speed below 0.2 m/s; at least 0.5 m retained progress toward goal; an actual receiver kick from at most 1.8 m; whole-ball goal crossing inside the posts. Transfer timeout 20 s, scoring timeout 24 s.

Every 5 ms physics sample checks defender-ball proximity/mouth possession and defender-attacker body clearance. Any interception, robot collision, invalid kick, possession loss before a valid release, field exit or timeout fails the full sequence. The defender continues to operate during scoring. Report initial and release lane clearance, receiver displacement, defender motion and minimum ball/robot distances, as well as each failure reason.

Target: at least 95% complete-sequence success on 200 reserved scenarios, with zero robot-collision or pre-kick possession-loss episodes. Interceptions count as failed episodes. Six consecutive preselected fresh demo scenarios retain every outcome. No qualification or demo observation enters training.

## Training plan and comparison

Contract `roboturk.defended_transfer.v1`: 52 inputs, seven actions. The previous 36-input transfer state is extended with defender positions/velocities in both attackers' frames, attack-relative geometry, the receiver's lateral-axis direction, current lane clearance and a stable-control flag. Actions and scripted role-handoff semantics match the previous play. The new actor starts from the previous transfer checkpoint, with additional input weights initialized to zero.

Training seed 67, one CPU thread. 300 new Webots teacher demonstrations (600000–600299), 100 balanced cloning epochs, 150 DAgger episodes (620000–620149), 60 refit epochs. Development seeds 610000–610059 select the warm start. Then 65,536 constrained PPO steps, learning rate 3e−7, training-observation regression, maximum normalized action change 0.01 and MSE 1e−6; projection learning rate 3e−6. Qualification 650000–650199 and demo 660000–660005 remain reserved until freezing. Report before/after PPO results without claiming a benefit if imitation already succeeds.

The previously qualified transfer policy failed **40/40** comparison scenarios (610000–610039): all were defender interceptions. The initial 1.85 m demonstration flank scored **37/40**, with three interceptions against the fastest defenders. The revised 2.2 m demonstration route scored **40/40**, with no possession losses, mean full-sequence duration 13.3195 s and mean pass distance 2.9529 m. All outcomes are retained. The defender and task criteria were unchanged during this attacker adjustment.

```sh
../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode defended-train --checkpoint policies/pass_receive_v2/policy.zip \
  --steps 65536 --seed 67 \
  --output runs/defended-v1 --timeout 1800

../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode defended-evaluate --checkpoint runs/defended-v1/policy.zip \
  --shoot-checkpoint policies/capture_dribble_shoot_v2/policy.zip \
  --episodes 200 --seed 650000 --output runs/defended-v1/qualification --timeout 900

../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode defended-demo --checkpoint runs/defended-v1/policy.zip \
  --shoot-checkpoint policies/capture_dribble_shoot_v2/policy.zip \
  --episodes 6 --seed 660000 --output runs/defended-v1/demo --timeout 300 --render
```

The native recording uses blue for the passer, cyan for the receiver/scorer and yellow for the opponent. Inference uses the same `PassReceiveScorePlay` and actual kick feedback as the previous milestone. Current evidence remains direct Webots Supervisor operation; full ROS integration needs applied-kick feedback in the message graph.

Approximate ideal body velocity, ODE contacts, spring/damper dribbler and fixed kicks; ground-truth state. No calibrated hardware, noisy vision, radio delay, free-ranging defenders, goalkeepers, variable kick power or complete SSL rules. [Webots Supervisor API](https://github.com/cyberbotics/webots/blob/master/docs/reference/supervisor.md) · [PPO implementation](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html).
