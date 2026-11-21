# Initial two-robot transfer experiment (not qualified)

This diagnostic experiment was superseded by the explicit-aiming-input v2 experiment. It was not submitted to reserved qualification or demo seeds. Do not present it as a reliable passing policy.

| Development stage (410000–410059) | Successful receptions | Failures | Possession losses |
| --- | ---: | --- | ---: |
| Cloned actor | 54/60 | 6 pass-release timeouts | 0 |
| DAgger refit | 37/60 | 23 pass-release timeouts | 0 |
| Final constrained PPO candidate | 56/60 | 4 pass-release timeouts | 0 |

All 64 PPO actor updates were rejected by the training-observation bounds. The observed evaluation difference is not evidence of a PPO actor improvement. These are transfer-only development results; complete sequences were only tested using the analytic transfer teacher composed with the learned scorer (40/40), preserved separately in teacher-development.json. Webots episode resets do not establish bitwise-identical cross-run outcomes for these marginal policies.

Training: centralized34-input/seven-action MLP,192×192 actor and critic,300 Webots demonstrations (400000–400299),80 cloning epochs,150 DAgger episodes (420000–420149;149 receptions during mixed teacher/student execution),50 refit epochs. Then65,536 constrained PPO steps, seed 47,oneCPUthread. All 300 teacher collection episodes received successfully. The best development warm start was bc.zip. No qualification or demo observations were used for training.

Artifacts retain all three checkpoints and their manifests, per-episode development/collection results, teacher baseline, actor-update diagnostics and stage hashes. Final candidate SHA-256: `f5d9b552b921c050f28e422ee22ed02c4671a525596bdaa4b01d36ea7d8b8b4a`. V2 starts from the included bc.zip and reuses the original training dataset; regenerate the dataset using the documented original trainer if needed (the large dataset remains local under runs/team-v1).

Contract: roboturk.pass_receive.v1. The passer begins holding the ball; the joint network commands passer vx/vy/omega/fire and receiver vx/vy/omega. Fixed1.8 m/s pass, receiver dribbler enabled, passer held after its actual release. The phase/role handoff is scripted. Ground-truth, unguarded, approximate Webots dynamics only.

Original source: [0f950437](https://github.com/kadiru/metu-ros-pkg/commit/0f95043714d4c2e4fa66175d1fe87efe94c2af38). See [training procedure and acceptance criteria](../../docs/TEAM_TRAINING.md).
