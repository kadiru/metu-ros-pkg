# Pressured pass reception

This specialist receives after the learned passer releases the ball. It approaches on the measured passer heading and leaves room for the monitored one-metre transfer. Within 0.48 m, it tracks the radial mouth error and yields to a fraction of the measured ball velocity. Its decisions use observed poses and ball velocity. Its training opponent is a scripted marker that closes continuously after the pass. The post-reception escape carry and neural finish come from the previous release.

A paired CPU search tested eight combinations of approach speed, velocity matching and mouth feedback, plus the previous receiver, on the same twelve Webots openings. The selection prioritized stable receptions, then goals, then fewer robot-proximity episodes. The selected parameters were 1.30 m/s outer approach speed, 0.40 ball-velocity weight and 4.0 per second mouth feedback. It secured 11/12 passes and scored 9/12 goals in training. This profile is opt-in for the two-attacker closing-marker play. [All training candidates](selection.json) · [Fresh results and Webots recordings](../../media/secure-pass/README.md)

```sh
python scripts/train_secure_receive.py --webots /path/to/webots --output runs/secure-fit
python scripts/run_webots.py --webots /path/to/webots --mode team-closing-evaluate --checkpoint policies/carpet_receive_v1/policy.zip --shoot-checkpoint policies/capture_dribble_shoot_v2/policy.zip --carry-profile policies/secure_receive_v1/policy.json --episodes 40 --seed 34912000 --timeout 1500 --output runs/secure-evaluation
```
