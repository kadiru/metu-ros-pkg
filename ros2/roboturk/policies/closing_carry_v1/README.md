# Carrying away from a closing marker

This profile controls the receiver after a stable catch. It combines the direction to goal with the direction away from the observed marker. The avoidance weight increases as the marker comes within 1.3 m of the ball. The robot turns toward goal while translating. The existing roller controller tracks the ball during the turn, and the frozen neural scorer takes over within 1.55 m. The 1.8 m/s pass, the 0.40 s stable reception test and the scorer's 3.5 m/s shot are the same as in the previous play.

Native rollout search tested four profiles on the same eight training openings. The selected combination, 1.5 m/s carry speed and 1.4 avoidance weight, scored 6/8. The other profiles scored 1/8, 1/8 and 4/8. This is a two-parameter search against one scripted opponent. The defender observes the ball and remains active through the shot. [Profile](policy.json) · [Training returns](selection.json) · [Fresh paired trials and footage](../../media/closing-press/README.md)

```sh
python scripts/train_closing_carry.py --webots /path/to/webots --output runs/carry-fit
python scripts/run_webots.py --mode team-closing-evaluate --checkpoint policies/carpet_receive_v1/policy.zip --shoot-checkpoint policies/capture_dribble_shoot_v2/policy.zip --carry-profile policies/closing_carry_v1/policy.json --episodes 40 --seed 33802000 --timeout 1800 --output runs/closing-team
```

On 40 fresh starts the fitted receiver and original finish scored once. This profile scored 13 times with the same 25 stable receptions.
