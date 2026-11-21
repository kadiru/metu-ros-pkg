# Defender-aware transfer v1 — failed development candidate

Not qualified for a demo or hardware. The old unobstructed transfer policy scored0/40 against the new analytic lane guard; the first1.85m demonstration route scored37/40, and the revised2.2m route scored40/40 with the frozen scorer. These full-sequence baselines are distinct from learned transfer results.

The52-input/seven-action body-frame actor was warm-started from `pass_receive_v2/policy.zip`. Webots teacher collection298/300, mixed DAgger145/150. Balanced cloning100epochs yielded23/60 development receptions, DAgger60epochs38/60, anchoredPPO65,536steps38/60. All64 actor updates accepted, no success improvement. There were no pre-pass possession-loss episodes in development; interceptions, field exits, invalid passes and timeouts remain in the raw records.

Training seed67; teacher600000–600299; DAgger620000–620149; development610000–610059. Reserved650000–650199 and660000–660005 were never used. Reproduce with [the original v1 source](https://github.com/kadiru/metu-ros-pkg/tree/41be952f41c313c7f6c2a6d6d18437645a2b41b6/ros2/roboturk) and the command in [training notes](../../docs/DEFENDER_TRAINING.md). All three stage models, manifests and results are retained here. The ignored `runs/defended-v1/demonstrations.npz` contains training-only observations used for the later v2 frame conversion. No development samples were added.

Scope: ground-truth approximate Webots dynamics; passer starts holding a ball; two attackers versus a fixed analytic moving midfield lane guard. No free-ranging opponent, goalkeeper, calibrated hardware or decentralized multi-agent RL.
