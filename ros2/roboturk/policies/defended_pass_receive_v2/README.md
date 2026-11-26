# Defender-aware transfer v2 — failed qualification

The frozen two-attacker play scored **185/200 (92.5%)**, below the predeclared95% target. There were11 defender interceptions, one transfer field exit, one transfer timeout, one invalid shot and one field exit during the scoring phase. Zero pre-kick possession-loss episodes. [Every outcome](qualification.json) · [Pre-test freeze](freeze.json). Reserved seeds750000–750199 were used once. No demo was recorded from the reserved760000–760005 set.

Development was59/60 transfer receptions after refit,58/60 after DAgger, and59/60 after constrained PPO. Full-play development scored58/60 (one interception and one field exit before shooting). All64 PPO actor updates were accepted; no incremental PPO success benefit is established. All stage checkpoints and records are retained.

The56-input/seven-action centralized network used attack-frame receiver vx/vy, robot-body passer vx/vy/omega/fire, and receiver omega. The interface transforms receiver velocities to its body frame, enables dribblers, uses a fixed1.8m/s pass and holds the passer after release. A scripted0.4s-stable-reception handoff invokes the unchanged learned scorer (fixed3.5m/s shot). No motion controller/teacher fallback at inference.

Warm start from [v1 DAgger](../defended_pass_receive_v1/dagger.zip), reusing only v1 training observations with action-frame conversion. Reset receiver translation output rows;100refit epochs.150new mixed DAgger episodes720000–720149 received148/150 (one interception, one field exit),60refit epochs, selected the refit checkpoint, then65,536 anchored PPO steps. Seed77, oneCPUthread, lr3e-7, training-observation regression with normalized action bounds MSE1e-6 andmax0.01. Development710000–710059. These are sequential experiments, not independent seed replications. [Reproduction and scope](../../docs/DEFENDER_TRAINING.md).

Transfer SHA256:`b630d35f37f4c1c020c734b27044a012e1ddccf97d37a96a36c38fea0d877199`. Frozen scorer SHA256:`6558b977dcdeb0001e9590b01ca1cfb9f3ec3fd16da1f43d17b8c3e6c9f6cc11`.

Passer starts with possession; initially blocked lane; independently scripted moving midfield guard, speed0.25–0.55m/s. Ground-truth approximate Webots physics. This is neither a free-ranging adversary benchmark nor hardware qualification. The failed qualification motivates a new play-layer receiving-point tactic with a new reserved test set; it is not silently replaced here.
