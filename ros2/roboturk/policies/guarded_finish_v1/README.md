# Keeper-aware finish after a pressured reception

The published Blue passer and secure receiver execute the transfer. After the measured 0.40 s stable handoff, Blue uses its existing escape carry in the approach. Near goal, it observes the keeper’s position and velocity, estimates clearance to two corners, commits to one and carries forward while turning onto the shot line. Shot release requires alignment and range. The finishing decision is a parameterized controller. The neural goalkeeper is frozen and the defender follows observed ball motion. [Implementation](../../src/roboturk_core/roboturk/guarded_finish.py)

Rollout training tested the previous finish plus six combinations of release range, corner offset and goalkeeper velocity prediction on the same ten Webots starts. Selection maximized goals, then minimized possession losses and total time for successful finishes. All six corner-aiming candidates scored eight goals on these training starts. The selected one had the shortest total time across its goals. The selected profile uses 1.55 m release range, a target 0.44 m from goal center, and 1.0 times the estimated flight time in the keeper-velocity projection. It scored 8/10 in training versus 5/10 for the previous team. This is single-sided CPU parameter training against fixed opponents. [Selection and all candidates](selection.json) · [Paired evaluation and films](../../media/guarded-finish/README.md)

```sh
python scripts/train_guarded_finish.py --webots /path/to/webots --output runs/guarded-fit
python scripts/run_webots.py --webots /path/to/webots --mode team-guarded-evaluate --checkpoint policies/carpet_receive_v1/policy.zip --carry-profile policies/guarded_finish_v1/policy.json --shoot-checkpoint policies/capture_dribble_shoot_v2/policy.zip --keeper-checkpoint policies/learned_opponents_v1/keeper/policy.zip --episodes 40 --seed 36012000 --timeout 1800 --output runs/guarded-check
```
