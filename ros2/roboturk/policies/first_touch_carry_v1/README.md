# Carry timing during a contested reception

This challenger uses the fitted carpet receiver, the published escape-carry parameters, and the existing neural scorer. It can begin carrying after one 20 ms control tick of stable receiving contact. During this phase it can only move the receiver with its dribbler enabled. A completed pass still requires 20 consecutive stable ticks with possession on every 2 ms substep and a valid pass of at least 1 m. A kick still requires the usual scorer handoff, controlled carry and valid shooting range.

In eight native Webots training openings the full-hold control and the one-tick challenger each scored five times. The full-hold control ranked first because it had two robot-proximity trials and the challenger had three. Four- and ten-tick delays scored one and zero times. The one-tick variant was the best earlier challenger. It was frozen for a separate paired test against the full-hold control. [Trial returns](selection.json) · [Independent comparison and matched replay](../../media/closing-press/README.md)

```sh
python scripts/train_closing_carry.py --webots /path/to/webots --fit-first-touch policies/closing_carry_v1/policy.json --seed 33820000 --episodes 8 --output runs/touch-fit
python scripts/run_webots.py --mode team-closing-evaluate --checkpoint policies/carpet_receive_v1/policy.zip --shoot-checkpoint policies/capture_dribble_shoot_v2/policy.zip --carry-profile policies/first_touch_carry_v1/policy.json --episodes 40 --seed 33822000 --timeout 1800 --output runs/first-touch
```

The training command writes the overall winner to `policy.json` and the best earlier timing to `challenger.json`. The released file in this directory is the earlier challenger.

On the 40 additional paired starts this challenger secured 21 passes and scored 17 goals. Waiting for the full hold secured 24 and scored 20.
