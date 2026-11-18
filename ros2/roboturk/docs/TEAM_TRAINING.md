# Two-robot pass, receive and score

The transfer policy is a centralized 34-input/seven-action MLP coordinating two teammates. A deterministic play assigns roles, holds the passer after its actual kick, and switches the receiver to the existing qualified scoring policy after 0.4 s of stable reception. Motion and fire decisions come from learned networks. The role switch is scripted; this is hierarchical skill composition, not decentralized multi-agent RL or an end-to-end learned strategy.

`PassReceiveScorePlay` is the core deployment API. Supply world state, role assignment, measured physics substeps and actual kick feedback. It rejects time rewinds/duplicate frames and holds on referee stops or missing perception. The current ROS message graph does not yet provide the kick feedback this play requires; full ROS integration remains future work. The Supervisor evaluator runs the same observation, actuator and handoff semantics.

## Scenario and acceptance target, declared before qualification

Two teammates on an unguarded field. The passer starts with a stationary ball at its dribbler; both headings are sampled over the full circle. Attack-relative passer x is −2.0 to −0.7 m and y is ±1.1 m. The receiver starts 1.3–2.4 m forward and ±0.9 m laterally from the passer. Both attack directions and both robot-ID role assignments are sampled.

Require 0.4 s stable control before an actual pass kick; launch-to-teammate distance and ball displacement before reception must each be at least 1.0 m. The receiver must establish 0.4 s continuous mouth possession with relative speed below 0.2 m/s. Then, without any physics reset, it must carry the ball at least 0.5 m closer to goal and score with an actual kick from within 1.8 m. The whole ball must cross the opponent goal line inside the posts. Any passer possession loss before release, receiver possession loss after stable reception and before shooting, robot collision, invalid kick, field exit or timeout fails. Transfer timeout16 s; scoring timeout24 s. Possession/collision checks use5 ms physics samples.

Target: at least95% complete-sequence success on200 reserved scenarios, zero collisions and zero possession-loss episodes before either valid release. Report transfer-only and complete-sequence outcomes separately. Keep every outcome in the six consecutive preselected demo scenarios.

## Training and seed separation

Training seed47, one CPU thread. 300 Webots teacher demonstrations (400000–400299),80 balanced cloning epochs,150 DAgger recovery episodes (420000–420149),50 refit epochs. Select a warm start using development410000–410059, then65,536 constrained PPO steps with the existing training-observation regression and actor-change bounds (MSE1e−6, max normalized change0.01). Qualification450000–450199 and demo460000–460005 are reserved until the policy is frozen. Report all stage results; do not claim a PPO benefit if cloning/DAgger already succeeds.

Observations: passer goal-conditioned ball state targeting the teammate (17), receiver ball state (12), relative heading cosine/sine (2), measured pass-release flag and two possession dwell counters (3). Actions: passer vx/vy/omega/fire and receiver vx/vy/omega. Skill/play actuator semantics retain the passer dribbler until firing, request a fixed1.8 m/s pass, hold the passer after release, and enable the receiver dribbler during reception. The receiver cannot kick until assigned the scoring skill. The reused scorer requests a fixed3.5 m/s shot. No teacher is invoked during checkpoint inference.

```sh
../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode team-train --steps 65536 --seed 47 --output runs/team-v1 --timeout 1200

../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode team-evaluate --checkpoint runs/team-v1/policy.zip \
  --shoot-checkpoint policies/capture_dribble_shoot_v2/policy.zip \
  --episodes 200 --seed 450000 --output runs/team-v1/qualification --timeout 600

../.venv/bin/python scripts/run_webots.py \
  --webots ../runtime/Webots-nightly.app/Contents/MacOS/webots \
  --mode team-demo --checkpoint runs/team-v1/policy.zip \
  --shoot-checkpoint policies/capture_dribble_shoot_v2/policy.zip \
  --episodes 6 --seed 460000 --output runs/team-v1/demo --timeout 240 --render
```

Development baseline: the analytic transfer teacher composed with the frozen learned scorer completed40/40 sequences (410000–410039), no possession losses, mean12.2235 s and2.008 m pass distance. This validates the curriculum and is not a learned transfer-policy result. Learned results are pending.

Scope: approximate Webots ideal body velocity, ODE ball contacts, spring/damper dribbler and edge-triggered kick; ground-truth observations; no opponents, goalkeeper, perception noise, radio latency or calibrated hardware. The passer starts holding the ball; arbitrary initial-ball acquisition by the pair is outside this benchmark.

API references: [Webots Supervisor](https://github.com/cyberbotics/webots/blob/master/docs/reference/supervisor.md), [Stable-Baselines3 PPO](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html).
